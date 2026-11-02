---
name: regenerate-ctags
description: In the trackinglib repository, Rebuild the .repo.tags symbol index at the repo root. Use at the start of a task per AGENTS.md's standing instruction to keep .repo.tags current, or whenever a rg lookup against .repo.tags seems stale.
---

Apply this workflow only in the trackinglib repository. Read its `AGENTS.md`
and run commands from the repository root.


# Regenerate .repo.tags

The ctags invocation is a fixed, checked-in script — do **not** re-derive or retype the flags by
hand. Run it from the repo root:

```bash
./scripts/regenerate_ctags.sh
```

This is the single source of truth for the invocation, also used by `.devcontainer/setup.sh` on
container creation. Do not tweak the `ctags` flags without updating the script (they're tuned to
keep the index small — see the rejected/prior variants noted in `.devcontainer/setup.sh`'s
comments above the call).
