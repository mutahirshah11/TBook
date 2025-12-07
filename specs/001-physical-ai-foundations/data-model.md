# Data Model / Content Hierarchy

**Feature**: `001-physical-ai-foundations`

## Entities

### Chapter
Represents the single MDX file `docs/part1/chapter1-foundations.mdx`.

| Field | Type | Description |
|-------|------|-------------|
| `frontmatter.id` | String | Unique ID `foundations-physical-ai` |
| `frontmatter.title` | String | "Chapter 1: Foundations of Physical AI" |
| `frontmatter.sidebar_label`| String | "1. Foundations" |
| `frontmatter.description` | String | SEO description |
| `content` | Markdown | The body text |

### Section Structure (Implicit)
The content MUST follow this hierarchy (enforced by TDD validator):

1.  **Motivation (The "Why")**
    *   Historical Context
    *   Limitations of disembodied AI
2.  **Definitions (The "What")**
    *   Physical AI vs Traditional AI
    *   Embodied Intelligence
3.  **Components (The "How")**
    *   Sensing, Actuation, Perception, Control
    *   Multidisciplinary nature
4.  **Summary**
    *   Key Takeaways

### Assets
| Type | Path Pattern | Description |
|------|--------------|-------------|
| Image | `static/img/part1/chapter1/*.png` | Static diagrams |
| Code | ` ```python ` blocks | Illustrative snippets |
