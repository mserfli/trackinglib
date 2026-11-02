# Checked construction and safety review

## Checked construction and migration

`FromList`, `StateVecFromList`, `MeasurementVecFromList`, and the explicit U/D
covariance helpers now return `tl::expected<Value, math::Errors>`. This is a breaking
return-type change: store the result, check `has_value()`, then access `value()`.
The same rule applies to constant lists. `FromLists` remains the preferred top-level
API and returns the first failure in vector, covariance, then concrete `TryCreate`
order. There are no error-discarding compatibility wrappers.

For a complete construction example with one user-facing result check:

```cpp
#include "trackingLib/motion/motion_model_cv.hpp"
#include <iostream>

int main()
{
  using Model = tracking::motion::MotionModelCV<tracking::math::FullCovarianceMatrixPolicy<float64>>;
  auto result = Model::FromLists(
      {30.0, 0.0, 10.0, 0.0},
      {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0},
       {0.0, 0.0, 1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}});
  if (!result.has_value())
  {
    std::cerr << "Model construction failed: " << static_cast<int>(result.error()) << '\n';
    return 1;
  }
  std::cout << result.value().getX() << '\n';
  return 0;
}
```

| Construction path | Checks and result semantics |
| --- | --- |
| Matrix, square, triangular, nested diagonal | Exact outer row count, then every row's column count, before copying. Errors are `invalid_initializer_list_row_count`, then `invalid_initializer_list_col_count`, independent of storage order. Triangular construction extracts the selected triangle; nested diagonal construction ignores off-diagonal values. |
| Flat vector or diagonal | Exact length; otherwise `invalid_initializer_list_size`. General matrices/vectors do not impose covariance positivity rules. |
| Full covariance list | Dimensions, then the existing symmetry predicate/tolerance; otherwise `matrix_not_symmetric`. Symmetric singular and indefinite matrices are supported at this boundary. Success alone does not prove PSD or PD. |
| Full list with factored policy | Checked full construction, then existing UDU decomposition with propagated errors. Non-positive pivots are clamped to epsilon by the existing algorithm; no new strict PD gate is added. |
| Explicit U/D | U dimensions, D length, extracted U's unit-upper predicate, then D's positive-definiteness predicate. Failed invariants return `matrix_not_unit_upper_triangular`, then `matrix_not_positive_definite`. The lower triangle of U is ignored; explicit D is never clamped. |
| Built-in model/observation `FromLists` | Vector first, covariance second, then `TryCreate` requires the resulting covariance to be positive definite. The sensor-pose overload follows the same order and forwards the pose. |

For example, a symmetric covariance with diagonal `[-1, 1]` succeeds at the full
covariance factory but is rejected by a full-policy observation's `TryCreate`.
The same list can produce a factored-policy observation after UDU clamps the first
pivot to epsilon. Policy selection can therefore change numeric acceptance as well
as storage. These checks preserve the existing predicates and tolerances; they do
not implement comprehensive NaN/Inf rejection (including the existing symmetry
predicate's non-finite limitations).

The named factories/helpers are `noexcept` for supported primitive types and built-in
policies/models. Extension-dependent paths use conditional exception specifications.
`noexcept` describes exception behavior, not finite inputs, construction success,
or freedom from assertions outside the checked guarantees. Direct constructors,
setters, `FromDiagonal`, and numerical algorithms retain their existing contracts.
Never access a failed result: the pinned `tl::expected` throws on failed `value()`
with exceptions enabled and can reach undefined behavior with exceptions disabled.
The [implementation plan](../plans/recent/complete-checked-fromlist-factories.md) records
the detailed scope and verification evidence.

## Safety-policy qualification and review status

Core algorithms use fixed-size storage and `tl::expected` instead of exceptions.
These project policies and passing tests do not certify AUTOSAR conformance. The
[official AUTOSAR C++14 guidelines](https://www.autosar.org/fileadmin/standards/R19-03/AP/AUTOSAR_RS_CPP14Guidelines.pdf)
are the rule source; the checked-factory review records these dispositions:

| Rule / concern | Disposition and remaining review |
| --- | --- |
| M0-3-2 | Factory error results are checked in propagation, tests, examples, and current documentation, including literals. Broader error handling remains outside this factory audit. |
| A15-4-4 | The named construction chain has audited exception specifications and compile-time `noexcept` checks for built-ins; extension paths are conditional. This is not an audit of all functions. |
| A7-2-2 / A7-2-4 | `math::Errors` explicitly uses `sint32`, preserving the prior `int` representation on supported targets and original error values; no enumerators have explicit initializers. |
| A15-0-4 / A15-0-5 | These rules prescribe exception-based handling for specified error categories. The deliberate `tl::expected` policy needs an applicability/deviation review; this documentation does not approve a deviation. AUTOSAR does not universally forbid exceptions. |
| C++17 and dependencies | Language tailoring from the C++14 rule set and assessment of the pinned external `tl::expected` implementation remain required. Failed-access behavior is a dependency limitation, so every result must be checked. No new dependency is introduced. |
| Verification coverage | Behavioral tests, standalone-header checks, compiler diagnostics, and configured clang-tidy checks are separate evidence, not a complete rule-by-rule assessment. `.clang-tidy` selects bugprone, Google, misc, modernize, performance, portability, and readability checks with exclusions and treats enabled warnings as errors. It is not a full AUTOSAR checker. A clang-tidy 18 retry found an unsupported `Disabled` configuration key; analysis with a temporary equivalent valid configuration reports outstanding diagnostics. Final run outcomes are recorded in the implementation plan. |

Uncovered rules, formal deviation approvals, language/dependency qualification, and
the broader direct-constructor/setter assertion audit remain outstanding. Existing
numeric tolerances, UDU clamping, and non-finite-input limitations still apply.


