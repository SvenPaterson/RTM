---
applyTo: "tools/**/*.py"
description: "Use when: editing Python host tooling in tools for capture, automation, or diagnostics."
---

# Python Tools Instructions

## Scope
Applies to host tooling scripts in tools.

## Rules
- Keep CLI behavior stable and explicit; prefer argparse flags over positional ambiguity.
- Maintain resilient serial handling (timeouts, retries, user-facing errors).
- Avoid silent failure paths; emit actionable diagnostics for operator setup issues.
- Keep output deterministic and log-friendly for post-run analysis.

## Required documentation updates
- Update [docs/PROJECT_HANDOFF.md](../../docs/PROJECT_HANDOFF.md) when host workflow or operator sequencing changes.
- Update [tools/requirements.txt](../../tools/requirements.txt) when dependencies change.

## Validation expectations
- Verify basic invocation paths (`--help` and primary scenario command) after meaningful CLI edits.
- Document any untested hardware-dependent path in the change summary.
