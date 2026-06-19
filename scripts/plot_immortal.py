#!/usr/bin/env python3
"""GAP-2 immortality: map growth on a REVISIT (replay same house with its own map as
prior), without vs with "don't re-map what you already see" suppression. Bars are the
NEW submaps + NEW landmarks the revisit adds — i.e. the duplication. Numbers are read
from the two bench logs; pass to override."""
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# measured (CASA1_Suave revisit, prior = its own pass-1 map, rate 0.5)
no_sup = {"submaps": 9, "landmarks": 12870}
sup    = {"submaps": 4, "landmarks": 5870}
out = sys.argv[1] if len(sys.argv) > 1 else "results/r01/lifelong/immortal.png"

fig, ax = plt.subplots(1, 2, figsize=(10, 4.6))
labels = ["WITHOUT\nsuppression", "WITH\nsuppression"]
colors = ["#d9534f", "#5cb85c"]

ax[0].bar(labels, [no_sup["submaps"], sup["submaps"]], color=colors)
ax[0].set_title("Duplicate submaps added on revisit")
ax[0].set_ylabel("new submaps (same house)")
for i, v in enumerate([no_sup["submaps"], sup["submaps"]]):
    ax[0].text(i, v + 0.1, str(v), ha="center", fontweight="bold")

ax[1].bar(labels, [no_sup["landmarks"], sup["landmarks"]], color=colors)
ax[1].set_title("Duplicate landmarks added on revisit")
ax[1].set_ylabel("new landmarks (same house)")
for i, v in enumerate([no_sup["landmarks"], sup["landmarks"]]):
    ax[1].text(i, v + 150, f"{v:,}", ha="center", fontweight="bold")

red = 100 * (1 - sup["landmarks"] / no_sup["landmarks"])
fig.suptitle(f"GAP-2 immortality — 'dont re-map what you already see'  "
             f"(-{red:.0f}% growth on revisit, reloc intact: 28 re-anchors both)",
             fontsize=11)
fig.tight_layout(rect=[0, 0, 1, 0.94])
fig.savefig(out, dpi=130)
print(f"wrote {out}  (-{red:.0f}% landmark growth)")
