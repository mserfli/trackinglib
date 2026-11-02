---
name: regenerate-media
description: In the trackinglib repository, Regenerate the example tracking GIFs in doc/media/ (build examples, run them to emit CSVs, render GIFs). Use when asked to rerun the examples, recreate/refresh the demo GIFs, or update doc/media after changing an example or the renderer.
---

Apply this workflow only in the trackinglib repository. Read its `AGENTS.md`
and run commands from the repository root.


# Regenerate the example GIFs

The entire pipeline is a fixed, checked-in script — do **not** re-derive the build/run/render steps
or run the examples by hand. Determinism and cost both depend on you doing exactly one thing:

1. Run the script from the repo root:

   ```bash
   ./scripts/regenerate_media.sh
   ```

2. Report the result of the `git status doc/media` line the script prints:
   - **No changes** → the GIFs are already up to date (the pipeline is deterministic; identical
     input renders identical bytes). Say so.
   - **Changed files** → the GIFs were refreshed; list which ones changed.

Routine regeneration requires no edits to GIFs, CSVs, or `render.py`. If the user
requests changes to examples or the renderer, make the authorized changes before
running this pipeline.

## Notes

- Byte-for-byte reproducibility holds only inside the pinned `.devcontainer` image
  (`trackinglib:latest`), which fixes the matplotlib/numpy/Pillow/font versions. Run this in that
  environment. A different Python/matplotlib stack may render pixel-identical scenes to different
  bytes.
- To verify without writing (e.g. in CI, same container): `./scripts/regenerate_media.sh --check`
  exits non-zero if any committed GIF is stale.
- To add or change an example, edit the `EXAMPLES` table in `scripts/regenerate_media.sh` — it is
  the single source of truth for the example → CSV → GIF mapping.
