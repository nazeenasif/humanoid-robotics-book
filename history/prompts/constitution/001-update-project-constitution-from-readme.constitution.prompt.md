---
id: 001
title: Update project constitution from README
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: [USER]
command: /sp.constitution --input-file README.md --output-file constitution.md --prompt "Read README.md and rewrite it as a Spec-Kit Plus constitution for this project. Keep all project details, rules, standards, constraints, and success criteria. Format properly in Markdown."
labels: ["documentation", "constitution"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Constitution updated and validated against README.md
---

## Prompt

```text
--input-file README.md --output-file constitution.md --prompt "Read README.md and rewrite  it as a Spec-Kit Plus constitution for this project. Keep all project details, rules, standards, constraints, and success criteria. Format properly in Markdown."
```

## Response snapshot

The project constitution has been updated to Version 2.0.0. This is a MAJOR version bump because the core principles of the project have been completely replaced with those defined in the `README.md`. No files require manual follow-up. Suggested commit message: `docs: amend constitution to v2.0.0 (update principles, standards, constraints, and success criteria from README)`

## Outcome

- ✅ Impact: Constitution updated and aligned with project README.
- 🧪 Tests: Constitution content reflects `README.md` and template standards.
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: None, task completed.
- 🧠 Reflection: Successfully updated the project constitution from `README.md` content, handling a major version bump and ensuring adherence to Spec-Kit Plus guidelines.

## Evaluation notes (flywheel)

- Failure modes observed: `create-phr.sh` script not found, switched to agent-native PHR creation.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Ensure `create-phr.sh` script is available or improve agent-native PHR creation robustness.
