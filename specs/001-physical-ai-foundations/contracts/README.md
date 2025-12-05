# Content Contract: Frontmatter Schema

**Type**: Docusaurus Frontmatter
**Format**: YAML

## Schema

```typescript
interface ChapterFrontmatter {
  // Unique identifier for the doc, used in URL and internal links
  id: string; // e.g., "foundations-physical-ai"

  // Display title of the chapter
  title: string;

  // Sidebar display label (shorter)
  sidebar_label: string;

  // SEO description and page metadata
  description: string;

  // Keywords for SEO
  keywords?: string[];

  // Order in the sidebar (optional if using sidebars.js auto-gen, but good for explicit ordering)
  sidebar_position?: number;
}
```

## Acceptance Criteria (TDD)

The content validator script MUST verify that:
1. `id` matches the filename slug.
2. `title` is present and non-empty.
3. `description` is present and > 20 characters.
