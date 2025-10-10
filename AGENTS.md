# Repository Agent Guidelines

This file provides helper conventions for automated agents contributing to the
RTM firmware review repository. The rules apply to the entire tree unless a
subdirectory defines its own `AGENTS.md` with more specific guidance.

## General workflow
- Prefer descriptive commit messages that explain *why* a change was made.
- Update or add documentation when you introduce new findings; do not leave the
  code review note (`CODE_REVIEW.md`) stale relative to captured logs.
- Keep line endings Unix-style (`LF`) and wrap prose at roughly 100 characters
  when practical.

## Documentation (`CODE_REVIEW.md` and other markdown files)
- Use level-2 headings for major topics so operators can skim the findings.
- Cite supporting evidence using the `【F:path†Lstart-Lend】` format with
  repo-relative paths and line spans when possible.
- When quoting serial captures, prefer fenced code blocks instead of inline
  snippets to keep timestamps readable.

## Logs (`logs/` directory)
- Name new captures with a concise description in snake_case and the `.txt`
  extension (for example: `2024-09-03_cold_boot.txt`).
- Store raw captures without editing timestamps or message ordering; add context
  in the code review document rather than inside the log files.

## Pull request messages
- Provide a bullet list summary of key findings.
- Note whether automated tests or reproductions were run; if none, call that
  out explicitly.

These conventions are intended to keep the ongoing investigation organized as we
triage the ClearCore/XPB boot and reset interactions.
