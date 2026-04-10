---
applyTo: "docs/**/*.md,README.md,CODE_REVIEW.md,test/**/*.md,test/README,.github/**/*.md"
description: "Use when: editing project documentation, handoff notes, or code review findings markdown."
---

# Documentation Instructions

## Scope
Applies to project documentation, review notes, test markdown docs, and instruction markdown files.

## Rules
- Keep documentation synchronized with functionality and test behavior changes.
- Prefer concise operator-focused steps over broad narrative when describing runbooks.
- Preserve stable section structure so future sessions can resume quickly.
- For findings documents, separate confirmed evidence from open hypotheses.

## CODE_REVIEW expectations
- Use level-2 headings for major finding groups.
- Cite evidence with the existing format:
  - `【F:path†Lstart-Lend】`
- Add or update a short session checkpoint whenever priorities or next actions change.

## Handoff expectations
- Keep [docs/PROJECT_HANDOFF.md](../../docs/PROJECT_HANDOFF.md) current with instruction layout, workflow changes, and test entry points.
- Ensure [README.md](../../README.md) continues to point to the active handoff and review docs.

## Validation expectations
- Verify internal links and command examples after edits.
- If documentation references a behavior change that has not yet been validated on hardware, call that out explicitly.
