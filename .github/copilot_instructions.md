# RTM Copilot Workspace Instructions

These instructions apply to the full repository.

## Instruction layering
- This file is always-on and should only contain cross-cutting repository policy.
- Path-specific guidance lives in [instructions/firmware.instructions.md](instructions/firmware.instructions.md), [instructions/python-tools.instructions.md](instructions/python-tools.instructions.md), [instructions/testing.instructions.md](instructions/testing.instructions.md), and [instructions/docs.instructions.md](instructions/docs.instructions.md).

## Primary goals
- Keep firmware, host tooling, and investigation notes in sync.
- Preserve fast handoff context so work can resume without rediscovery.
- Treat documentation updates as required deliverables, not optional cleanup.

## Documentation maintenance requirement
- Any change to project functionality MUST include documentation updates in the same change.
- Any change to test utilities, test scenarios, or validation thresholds MUST include documentation updates in the same change.
- At minimum, update the most relevant files among [README.md](../README.md), [test/README](../test/README), [docs/ttl_sniffer_debug.md](../docs/ttl_sniffer_debug.md), [docs/PROJECT_HANDOFF.md](../docs/PROJECT_HANDOFF.md), and [CODE_REVIEW.md](../CODE_REVIEW.md).
- If behavior changed but no docs were updated, treat the task as incomplete.

## Workflow conventions
- Use descriptive commit messages that explain why a change was made.
- Keep line endings as LF and wrap prose near 100 columns when practical.
- When adding findings from logs, ensure [CODE_REVIEW.md](../CODE_REVIEW.md) is updated in the same working session.

## Evidence and logging
- Check [test/log/](../test/log/) for recent captures before concluding a root cause.
- If additional captures are needed, ask the operator to enable log capture and collect a fresh run.
- Use fenced code blocks when quoting log snippets to preserve timestamps and readability.
