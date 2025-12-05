# Research: Foundations of Physical AI Chapter

**Feature**: `001-physical-ai-foundations`
**Date**: 2025-12-05

## Decision 1: Content Architecture

- **Decision**: Single MDX file per chapter (`docs/part1/01-foundations.mdx`).
- **Rationale**: Docusaurus generates a table of contents automatically. Splitting a single chapter into multiple files fragments the reading experience for a textbook.
- **Alternatives Considered**:
    - *Sub-directory with multiple files*: Better for massive documentation, but overkill for a textbook chapter.

## Decision 2: Interactive Diagrams

- **Decision**: Use **Mermaid.js** for flowcharts/architecture diagrams (native Docusaurus support) and **Standard React Components** for interactive simulations if needed later. For this foundational chapter, static images and Mermaid will suffice for the "Motivation -> What -> How" flow.
- **Rationale**: Low maintenance, high compatibility.
- **Alternatives Considered**:
    - *External iframe embeddings*: Harder to maintain, dependency risk.
    - *Custom Canvas/WebGL*: Too high effort for "Foundations" chapter.

## Decision 3: TDD Strategy for Content

- **Decision**: Implement a **Structure Validation Script** (Node.js) as the "Test".
- **Rationale**: The Constitution requires TDD. For content, "Test" means verifying the existence of files, required frontmatter, specific section headers (matching the "Motivation First" spec), and presence of placeholders for required figures/code.
- **Alternatives Considered**:
    - *Manual Review*: Violates "Automated" spirit of TDD.
    - *LLM-based critique*: Good for quality, but hard to make deterministic "Red/Green".

## Decision 4: Asset Management

- **Decision**: Store images in `static/img/part1/chapter1/`.
- **Rationale**: Standard Docusaurus pattern for static assets.
