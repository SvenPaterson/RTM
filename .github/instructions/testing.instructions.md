---
applyTo: "test/**/*"
description: "Use when: editing test harness scripts, test docs, or captured-test workflows."
---

# Testing Instructions

## Scope
Applies to test harness scripts, wrappers, and test documentation.

## Rules
- Keep test scenarios reproducible with explicit defaults and logged configuration.
- Preserve compatibility wrappers unless intentionally deprecating with documented migration.
- Ensure logs capture command invocation, timing assumptions, and final status.
- Prefer additive scenario coverage over breaking existing operator workflows.

## Required documentation updates
- Update [test/README](../../test/README) for any new mode, flag, default, threshold, or workflow change.
- Update [docs/PROJECT_HANDOFF.md](../../docs/PROJECT_HANDOFF.md) when test flow or runbook steps change.
- Update [CODE_REVIEW.md](../../CODE_REVIEW.md) when test outcomes materially affect findings or priorities.

## Validation expectations
- Run or simulate the touched scenario path when possible.
- If execution is blocked by hardware availability, include the exact command sequence for operator verification.
