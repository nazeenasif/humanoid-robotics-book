<!-- Sync Impact Report:
Version change: 1.0.0 → 2.0.0
Modified principles: Replaced existing principles with those from README.md
Added sections: Key Standards, Constraints, Success Criteria, AI Collaboration Rules
Removed sections: None
Templates requiring updates:
  ✅ .specify/templates/plan-template.md
  ✅ .specify/templates/spec-template.md
  ✅ .specify/templates/tasks-template.md
  ✅ .claude/commands/sp.adr.md
  ✅ .claude/commands/sp.analyze.md
  ✅ .claude/commands/sp.checklist.md
  ✅ .claude/commands/sp.clarify.md
  ✅ .claude/commands/sp.constitution.md
  ✅ .claude/commands/sp.git.commit_pr.md
  ✅ .claude/commands/sp.implement.md
  ✅ .claude/commands/sp.phr.md
  ✅ .claude/commands/sp.plan.md
  ✅ .claude/commands/sp.specify.md
  ✅ .claude/commands/sp.tasks.md
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation Constitution

## Core Principles

### I. Accuracy and Reliability
All technical explanations MUST be accurate and reliable.

### II. Clarity and Accessibility
Content MUST be clear and accessible for students, beginners, and self-learners.

### III. Consistent Structure
All book chapters MUST maintain a consistent structure.

### IV. Modularity
Each chapter MUST function both independently and as part of the whole.

### V. AI-Assisted Productivity
AI assistance MUST be used for productivity without compromising originality.

## Key Standards

### I. Technical Validation
All technical statements MUST be validated through reputable documentation (e.g., MDN, React docs, Docusaurus docs, GitHub Docs).

### II. Writing Style
The writing style MUST be beginner-friendly, conversational, and educational.

### III. Code Examples
Code examples MUST be tested or logically correct, and verified via Claude Code.

### IV. Formatting
Formatting MUST follow Docusaurus standards (MDX + folder structure).

### V. Chapter Structure
Every chapter MUST include: introduction, concepts, examples, summary, and optional exercises.

## Constraints

### I. Chapter Count
The book MUST have a minimum of 10 chapters.

### II. Chapter Length
Each chapter MUST be approximately 800–1500 words.

### III. Original Content
All content MUST be original (AI-generated but edited for uniqueness).

### IV. Docusaurus Compatibility
The book MUST be fully compatible with Docusaurus v3.

### V. Build Process
The project MUST build without errors (`npm run build`).

### VI. Deployment
The book MUST deploy successfully to GitHub Pages.

## Success Criteria

### I. Build Health
Docusaurus build MUST pass with zero errors or warnings.

### II. Content Quality
Content MUST be logically correct, readable, and well-structured.

### III. Code Functionality
All code snippets MUST run or make sense conceptually.

### IV. Rendering Quality
The book MUST render cleanly (navigation, sidebar, search).

### V. Public Accessibility
Final GitHub Pages deployment MUST be accessible publicly.

### VI. Educational Value
The overall book MUST meet educational value and clarity requirements.

## AI Collaboration Rules

### I. Structure and Standards
Spec-Kit Plus MUST be used to maintain structure, enforce standards, and auto-generate specs.

### II. Content Creation
Claude Code MUST be used for writing, editing, code generation, and content refinement.

### III. Factual Accuracy
No auto-hallucinated technologies; factual checks are REQUIRED when unsure.

### IV. Consistency
Consistent tone, formatting, examples, and glossary MUST be maintained.

## Development Workflow

Code review requirements, testing gates, deployment approval process, etc.

## Governance
All PRs/reviews MUST verify compliance; Complexity MUST be justified; Use CLAUDE.md for runtime development guidance.

**Version**: 2.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
