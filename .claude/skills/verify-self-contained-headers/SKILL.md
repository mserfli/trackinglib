---
name: verify-self-contained-headers
description: Verify every header under include/trackingLib/ compiles standalone (no prerequisite includes) via the header_tests CMake target. Use when checking that headers compile standalone, after adding or moving a header, or before a PR that touches include/.
---

# Verify self-contained headers

The whole check is a fixed, checked-in script — do **not** re-derive the configure/build/ctest
steps by hand. Determinism and cost both depend on you doing exactly one thing:

1. Run the script from the repo root:

   ```bash
   ./scripts/verify_self_contained_headers.sh
   ```

2. Report the result:
   - **All green** → every header compiles standalone. Say so.
   - **Failures** → list which header(s) failed and the compiler error(s) ctest reported.

That's the whole job. Do not tweak the CMake flags or the ctest filter.

## Notes

- This mirrors CI's `verify-self-contained-headers` job in `.github/workflows/build-and-test.yml`,
  which runs the same script.
- Every `.h`/`.hpp` under `include/trackingLib/` must be includable with no prerequisite includes
  — this is an AGENTS.md-documented constraint, not just a style preference.
