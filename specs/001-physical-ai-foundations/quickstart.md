# Quickstart: Developing Chapter 1

**Feature**: `001-physical-ai-foundations`

## Prerequisites

1.  **Node.js** (v18+)
2.  **npm**

## Setup

1.  Install dependencies:
    ```bash
    npm install
    ```

## Development Workflow (TDD)

1.  **Run the Content Validator (Expect Failure)**
    ```bash
    # (We will create this script in the implementation phase)
    node scripts/validate-chapter-structure.js --chapter 01-foundations
    ```
    *Expectation*: Fails because file doesn't exist or is empty.

2.  **Create the Content File**
    *   Create `docs/part1/chapter1-foundations.mdx`.
    *   Add Frontmatter.
    *   Add Section Headers (Motivation, Definitions, etc.).

3.  **Run the Content Validator (Expect Success)**
    ```bash
    node scripts/validate-chapter-structure.js --chapter 01-foundations
    ```
    *Expectation*: Passes.

4.  **Start Docusaurus Preview**
    ```bash
    npm start
    ```
    *   Navigate to `http://localhost:3000/docs/part1/chapter1-foundations`.

5.  **Fill Content**
    *   Write text.
    *   Add images to `static/img/part1/chapter1/`.
    *   Add code blocks.
