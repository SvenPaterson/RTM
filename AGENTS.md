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
- Log caputuring can be toggled on by the human user and will be automatically stored 
  in the logs folder with following format "COM#_YYYY_MM_DD.HH.MM.SSS.txt". Always check
  the logs folder for recent captures to reference with recent bugs.
- Prompt the user to turn on log capturing if it would be pertinent to the task at hand
  or if it looks like they forgot to do so

## Pull request messages
- Provide a bullet list summary of key findings.
- Note whether automated tests or reproductions were run; if none, call that
  out explicitly.

## Reply to User
- when providing response, if there is a useful debug step the user can perform to
  verify your change then let them know. This could be a specific sequence of actions to
  ellicit a specific system response to test changes.
- if the user is on WSL for windows they will need to be reminded to run the following commands
  in an elevated PowerShell window and leave it open while working:
      usbipd attach --wsl --auto-attach --hardware-id 2890:8022
      usbipd attach --wsl --auto-attach --hardware-id 2890:0022
      usbipd attach --wsl --auto-attach --hardware-id 2341:0058

These conventions are intended to keep the ongoing investigation organized as we
triage the ClearCore/XPB boot and reset interactions.
