<!--
Version change: 1.1.1 -> 1.2.0
Modified principles:
- Added AI Responsibility principle
- Added Data Privacy & Security principle
- Added AI Ethics principle
- Added Performance & Scalability principle
- Updated Key Standards to include AI-specific requirements
- Updated Testing & Verification to include AI-specific tests
- Updated Governance to include AI model governance
Added sections: AI Responsibility, Data Privacy & Security, AI Ethics, Performance & Scalability
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (Constitution Check section needs to be updated to include AI-specific checks)
- .specify/templates/spec-template.md: ⚠ pending (Requirements section needs to be updated to include AI-specific requirements)
- .specify/templates/tasks-template.md: ⚠ pending (Task types need to be updated to include AI-specific tasks)
Follow-up TODOs:
- Update all plan templates to include AI governance checks
- Update spec templates to include AI-specific requirements
- Update task templates to include AI-specific implementation tasks
-->
# Digital Textbook on Physical AI & Humanoid Robotics (Docusaurus) Constitution

## Core Principles

### Accuracy
All claims must be verified against primary sources whenever possible.

### Clarity
Writing must target Flesch-Kincaid grade 10-12, clear for readers with a computer science or AI background.

### Reproducibility
All references, examples, and diagrams must be traceable.

### Rigor
MUST prioritize peer-reviewed or authoritative sources; ensure technical and scientific correctness.

### Integrity
Zero tolerance for plagiarism or fabricated content.

### AI Responsibility
All AI-generated content must be clearly identified as such; the system must cite sources for retrieved information; hallucination detection and prevention mechanisms must be implemented; the system must refuse to generate harmful or inappropriate content.

### Data Privacy & Security
All user data and documents must be handled with appropriate privacy protections; sensitive information must be identified and handled according to data protection regulations; document access and usage must be logged for audit purposes; user consent must be obtained for data processing where required.

### AI Ethics
Bias detection and mitigation measures must be implemented in retrieval and generation; the system must handle queries across different demographic groups fairly and inclusively; the system must implement content filtering to prevent generation of harmful responses; AI decisions must be explainable and interpretable where possible.

### Performance & Scalability
95% of queries must return results within 3 seconds; the system must handle expected concurrent users without degradation; memory usage and computational efficiency must be optimized; throughput requirements (queries per second) must be met under expected load.

### Test-Driven Development
Test-Driven Development (TDD) is a mandatory practice. Tests MUST be written before implementation, user-approved, and initially fail. A strict Red-Green-Refactor cycle MUST be enforced.

### Key Standards
- All factual claims must be traceable to sources.
- Citation format: APA style.
- Minimum 50% of sources must be peer-reviewed or authoritative.
- Writing clarity: Flesch-Kincaid grade 10-12; active voice ≥ 75%.
- Academic integrity: 0% plagiarism tolerance.
- Figures, tables, diagrams, and code snippets must be accurate, labeled, and verified.
- Digital formatting: Markdown compatible with Docusaurus; all hyperlinks, images, and embeds must function correctly.
- Response accuracy: ≥95% of factual claims must be verifiable against source documents.
- Source attribution: All generated responses must cite their source documents when applicable.
- Safety standards: The system must implement content filtering to prevent generation of harmful responses.
- Model performance: All AI models must meet defined accuracy, precision, and recall metrics.

## Constraints

- Word count: flexible per chapter but must meet book standards.
- Format: Markdown for Docusaurus deployment.
- All work must comply with the Constitution; no deviation allowed.
- AI systems must include observability, logging, and monitoring capabilities.
- Models must be evaluated for bias, fairness, and safety before deployment.

## Testing & Verification

- Verify all claims against authoritative sources.
- Run plagiarism check on all content before publication.
- Check readability scores and active voice percentage.
- Review diagrams, tables, figures, and code snippets for accuracy and proper labeling.
- Validate all Markdown links, images, and embeds in Docusaurus preview.
- Test for hallucination: verify that generated content is supported by source documents.
- Test for bias: evaluate responses across different demographic groups and topics.
- Test for safety: verify the system handles inappropriate queries appropriately.
- Test for performance: validate response times and throughput under expected load.
- Test for long-term conversation coherence and consistency.

## Governance

- **Amendment Process**: Amendments to this constitution MUST follow a formal review and approval process, documented with rationale.
- **Compliance Review**: Adherence to these constitutional principles MUST be reviewed regularly (e.g., quarterly) to ensure ongoing alignment and effectiveness.
- **Enforcement**: All contributors are responsible for upholding these principles. Violations will be addressed through established project governance procedures.
- **AI Model Governance**: All AI models and their datasets must undergo approval before deployment, including bias testing, safety evaluation, and performance validation.

**Version**: 1.2.0 | **Ratified**: 2025-12-01 | **Last Amended**: 2025-12-08