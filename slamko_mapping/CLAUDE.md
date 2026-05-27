# slamko_mapping — submaps, persistence, map-server contract

Part of **slamko** — read [`../CLAUDE.md`](../CLAUDE.md) + [`../MASTER_PLAN.md`](../MASTER_PLAN.md)
+ [`../docs/DECOUPLING.md`](../docs/DECOUPLING.md) first.

**Role:** owns the map. `SubMap` lifecycle (build, seal, persist, load),
sparse + optional dense (occupancy) payloads, and the **map-server contract**
(DigiForest D3.1/D4.x): the map as a published data-format + API → multi-robot,
multi-session, swappable. Cross-session re-anchor on load.

**Implements / owns:** `SubMap` persistence; map-server API. **Depends on:**
slamko_core (+ slamko_msgs for the API). **Status:** planned. **Phase P4.**

**Starting cold here?** Read the 3 hub docs + this, then plan mode →
`docs/PLAN_P4_mapping.md`.

**Doc rule:** green tests → update `docs/STATUS.md` + stamps → commit together.
