# Phase AA failure record

Algorithmic: corrected replay rejected 51 frames under the historical combined line-search/trust reason; 226 accepted frames stopped at the iteration limit rather than true convergence. Legacy replay rejected 47 under that combined reason and 9 for low support. These are preserved outcomes, not discarded samples or proof of one root cause. No new correction was validated.

Reproducibility: old scan reference and pose-cache/clamp scheduling are incompletely recoverable; height improvement and independent accuracy were not separately measured. Exact old/new single-patch causality is unsupported.

Tooling: the legacy log initially inherited a reference Source identity field; separate hash bindings preserve actual input identity and a metadata-only CLI fix labels it explicitly. No numeric results changed.

Delivery: the preceding push attempt failed with `could not read Username for https://github.com`. The remote address/history was preserved and local commits retained. The current delivery attempt must report its actual outcome; no force-push is permitted.
