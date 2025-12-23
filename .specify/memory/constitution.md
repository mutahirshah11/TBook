<!--
Version change: 1.3.0 -> 1.4.0
Modified principles:
- Added User Authentication & Profile Management: Enable Better-Auth signup/signin and secure user profile handling
- Added Personalization & User Context: Capture and utilize user software/hardware background for content adaptation
- Added Translation & Localization: Enable chapter content translation to Urdu via user interface
Added sections: User Authentication & Profile Management, Personalization & User Context, Translation & Localization
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (Constitution Check needs to enforce new auth/personalization principles)
- .specify/templates/spec-template.md: ⚠ pending (Spec template needs auth/personalization sections)
- .specify/templates/tasks-template.md: ⚠ pending (Tasks must reflect auth/personalization/translation requirements)
Follow-up TODOs:
- Implement Better-Auth integration across all user-facing components
- Add user profile collection and management system
- Implement content adaptation based on user profiles
- Add Urdu translation functionality with appropriate UI controls
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

### Strict Test-Driven Development & Verification
**Test-Driven Development (TDD) is STRICTLY MANDATORY.** No feature implementation may begin until failing tests are written and approved. Verification is non-negotiable: every feature must include automated tests that prove correctness. Manual verification is insufficient. ALL pull requests and feature completions MUST be accompanied by passing test suites covering unit, integration, and edge cases.

### User Authentication & Profile Management
MUST implement Better-Auth for secure user signup and signin functionality; user profiles must be securely stored and managed with appropriate privacy protections; user credentials must be encrypted and never stored in plain text; user profile data must include software and hardware background information for personalization purposes; authentication tokens must be properly managed with secure session handling; user consent must be obtained for profile data collection and processing; user data must be accessible only to authorized users and with appropriate access controls.

### Personalization & User Context
MUST capture and store user software and hardware background information during registration and profile updates; the book and RAG chatbot MUST adapt content based on user profile information including technical expertise, software/hardware environment, and learning preferences; personalization algorithms must be designed to enhance user experience without compromising content accuracy; user context data must be updated regularly to reflect changes in user environment and preferences; content adaptation must maintain technical accuracy while optimizing for user's specific context and needs.

### Translation & Localization
MUST provide functionality for logged-in users to translate chapter content into Urdu via a button at the start of each chapter; translation services must be integrated securely and efficiently; translated content must maintain technical accuracy and context; user interface elements for translation must be intuitive and accessible; translation functionality must be available only to authenticated users; translated content must be properly formatted and styled to match original chapter presentation.

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
- Authentication systems must follow security best practices and industry standards.
- User profile data must comply with data protection regulations (GDPR, CCPA, etc.).
- Translation functionality must maintain content accuracy and user privacy.

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
- Test user authentication flows for security and usability.
- Test user profile collection and management functionality.
- Test content personalization based on user profiles.
- Test translation functionality for accuracy and performance.
- Test Urdu translation quality and formatting preservation.

## Governance

- **Amendment Process**: Amendments to this constitution MUST follow a formal review and approval process, documented with rationale.
- **Compliance Review**: Adherence to these constitutional principles MUST be reviewed regularly (e.g., quarterly) to ensure ongoing alignment and effectiveness.
- **Enforcement**: All contributors are responsible for upholding these principles. Violations will be addressed through established project governance procedures.
- **AI Model Governance**: All AI models and their datasets must undergo approval before deployment, including bias testing, safety evaluation, and performance validation.
- **Authentication Governance**: User authentication systems must undergo security review and penetration testing before deployment; access controls must be regularly audited.
- **Data Governance**: User profile and personalization data must be governed by privacy policies and data protection regulations; data retention and deletion policies must be enforced.

**Version**: 1.4.0 | **Ratified**: 2025-12-01 | **Last Amended**: 2025-12-23