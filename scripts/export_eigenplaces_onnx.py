#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Export EigenPlaces (ResNet18, 512-D, MIT) → ONNX for slamko's TensorRT VPR front-end.

WHY: XFeat-64 local descriptors carry no place-level signal (proven: a genuine loop
return is indistinguishable from places 80 m away). EigenPlaces is a learned GLOBAL
descriptor (MIT) that DOES discriminate places — validated on TUM VI magistrale1:
Recall@5 = 1.0 (every return retrieves the start room), 0/30 false matches. This script
ships the model as ONNX so slamko_vio runs it on the existing ONNX→TensorRT path
(mirrors xfeat.onnx), one 512-D vector per keyframe → cosine retrieval → the existing
XFeat+PnP geometric verifier.

The ONNX is the NETWORK ONLY: input 1×3×H×W (already grayscale→3ch replicated, resized,
ImageNet-normalized — the C++ wrapper does that, like xfeat.cpp::process_input), output
1×512 (L2-normalized). Fixed input shape (static engine, like xfeat.cpp:105).

Architecture rebuilt in pure torch (torchvision's C++ ops ABI-mismatch torch 2.9+cu130);
keys verified against the released state_dict (zero missing/unexpected). Weights:
  torch.hub.load("gmberton/eigenplaces","get_trained_model",backbone="ResNet18",fc_output_dim=512)
cached at ~/.cache/torch/hub/checkpoints/ResNet18_512_eigenplaces.pth (or pass --weights).

Usage:
  python3 scripts/export_eigenplaces_onnx.py \
      --weights /tmp/torchhub/checkpoints/ResNet18_512_eigenplaces.pth \
      --out slamko_vio/models/eigenplaces.onnx --size 512
"""
import argparse
import sys
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


class BasicBlock(nn.Module):
    expansion = 1
    def __init__(self, inp, out, stride=1, downsample=None):
        super().__init__()
        self.conv1 = nn.Conv2d(inp, out, 3, stride, 1, bias=False)
        self.bn1 = nn.BatchNorm2d(out)
        self.relu = nn.ReLU(inplace=True)
        self.conv2 = nn.Conv2d(out, out, 3, 1, 1, bias=False)
        self.bn2 = nn.BatchNorm2d(out)
        self.downsample = downsample
    def forward(self, x):
        idn = x
        o = self.relu(self.bn1(self.conv1(x)))
        o = self.bn2(self.conv2(o))
        if self.downsample is not None:
            idn = self.downsample(x)
        return self.relu(o + idn)


def make_layer(inp, out, blocks, stride):
    downsample = None
    if stride != 1 or inp != out:
        downsample = nn.Sequential(nn.Conv2d(inp, out, 1, stride, bias=False),
                                   nn.BatchNorm2d(out))
    layers = [BasicBlock(inp, out, stride, downsample)]
    for _ in range(1, blocks):
        layers.append(BasicBlock(out, out))
    return nn.Sequential(*layers)


def resnet18_backbone():
    return nn.Sequential(
        nn.Conv2d(3, 64, 7, 2, 3, bias=False), nn.BatchNorm2d(64), nn.ReLU(inplace=True),
        nn.MaxPool2d(3, 2, 1),
        make_layer(64, 64, 2, 1), make_layer(64, 128, 2, 2),
        make_layer(128, 256, 2, 2), make_layer(256, 512, 2, 2))


class GeM(nn.Module):
    def __init__(self, p=3, eps=1e-6):
        super().__init__(); self.p = nn.Parameter(torch.ones(1) * p); self.eps = eps
    def forward(self, x):
        return F.avg_pool2d(x.clamp(min=self.eps).pow(self.p),
                            (x.size(-2), x.size(-1))).pow(1.0 / self.p)


class L2Norm(nn.Module):
    def __init__(self, dim=1):
        super().__init__(); self.dim = dim
    def forward(self, x): return F.normalize(x, p=2.0, dim=self.dim)


class Flatten(nn.Module):
    def forward(self, x): return x[:, :, 0, 0]


class EigenPlaces(nn.Module):
    def __init__(self, dim=512):
        super().__init__()
        self.backbone = resnet18_backbone()
        self.aggregation = nn.Sequential(L2Norm(), GeM(), Flatten(),
                                          nn.Linear(512, dim), L2Norm())
    def forward(self, x): return self.aggregation(self.backbone(x))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--size", type=int, default=512)
    ap.add_argument("--dim", type=int, default=512)
    a = ap.parse_args()

    model = EigenPlaces(a.dim)
    sd = torch.load(a.weights, map_location="cpu")
    missing, unexpected = model.load_state_dict(sd, strict=False)
    miss = [k for k in missing if "num_batches_tracked" not in k]
    unexp = [k for k in unexpected if "num_batches_tracked" not in k]
    if miss or unexp:
        sys.exit(f"STATE DICT MISMATCH missing={miss} unexpected={unexp}")
    model.eval()

    dummy = torch.randn(1, 3, a.size, a.size)
    # Force the legacy TorchScript exporter (dynamo=False): torch 2.9's dynamo exporter
    # mis-translated GeM's pow-with-learnable-parameter (parity cos 0.47). Legacy is the
    # battle-tested path for plain CNNs and what xfeat.onnx was exported with.
    torch.onnx.export(
        model, dummy, a.out, input_names=["image"], output_names=["descriptor"],
        opset_version=13, dynamic_axes=None, dynamo=False)  # static shape
    print(f"wrote {a.out}  (input 1x3x{a.size}x{a.size} → {a.dim}-D)")

    # Numerical parity check: ONNX runtime vs torch on a random input (the input-format
    # risk — must match or recall collapses silently downstream).
    with torch.no_grad():
        ref = model(dummy).numpy()[0]
    try:
        import onnxruntime as ort
        sess = ort.InferenceSession(a.out, providers=["CPUExecutionProvider"])
        onnx_out = sess.run(["descriptor"], {"image": dummy.numpy()})[0][0]
        cos = float(ref @ onnx_out / (np.linalg.norm(ref) * np.linalg.norm(onnx_out)))
        maxabs = float(np.max(np.abs(ref - onnx_out)))
        print(f"ONNX parity: cos(torch,onnx)={cos:.8f}  max|Δ|={maxabs:.2e}  "
              f"|ref|={np.linalg.norm(ref):.4f} (≈1 ⇒ L2-normed)")
        if cos < 0.9999:
            sys.exit("PARITY FAIL — ONNX output diverges from torch")
    except ImportError:
        print("onnxruntime not installed — skipped parity check (install to verify)")


if __name__ == "__main__":
    main()
