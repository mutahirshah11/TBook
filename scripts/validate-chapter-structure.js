const fs = require('fs');
const path = require('path');

const chapterArg = process.argv.find(arg => arg.startsWith('--chapter='));
if (!chapterArg) {
  console.error('Error: Missing --chapter argument. Usage: node scripts/validate-chapter-structure.js --chapter=01-foundations');
  process.exit(1);
}

const chapterName = chapterArg.split('=')[1];
// Mapping chapter short code to filename
const fileMap = {
  '01-foundations': 'part1/chapter1-foundations.mdx'
};

const relativePath = fileMap[chapterName];
if (!relativePath) {
  console.error(`Error: Unknown chapter code "${chapterName}". Available: ${Object.keys(fileMap).join(', ')}`);
  process.exit(1);
}

const filePath = path.join(__dirname, '..', 'docs', relativePath);

console.log(`Validating ${relativePath}...`);

if (!fs.existsSync(filePath)) {
  console.error(`FAIL: File not found at ${filePath}`);
  process.exit(1);
}

const content = fs.readFileSync(filePath, 'utf-8');

// 1. Check Frontmatter
const frontmatterRegex = /^---\r?\n([\s\S]*?)\r?\n---/;
const match = content.match(frontmatterRegex);

if (!match) {
  console.error('FAIL: Missing frontmatter (YAML block at top of file)');
  process.exit(1);
}

const yaml = match[1];
const hasId = /id:\s*foundations-physical-ai/.test(yaml);
const hasTitle = /title:\s*.+/ .test(yaml);
const hasDesc = /description:\s*.{20,}/.test(yaml); // Description > 20 chars

if (!hasId) console.error('FAIL: Frontmatter missing or incorrect "id" (expected "foundations-physical-ai")');
if (!hasTitle) console.error('FAIL: Frontmatter missing "title"');
if (!hasDesc) console.error('FAIL: Frontmatter missing or too short "description" (>20 chars)');

if (!hasId || !hasTitle || !hasDesc) {
  process.exit(1);
}

// 2. Check Section Headers (Motivation, Definitions, Components, Summary)
// Allow ## or ### or just checking for existence of these phrases in headings
const requiredHeaders = [
  'Motivation',
  'Definitions',
  'Components',
  'Summary'
];

const missingHeaders = [];
requiredHeaders.forEach(header => {
  // Regex matches # Motivation, ## Motivation, ### Motivation... case insensitive
  const headerRegex = new RegExp(`^#{1,6}\\s+.*${header}`, 'm');
  if (!headerRegex.test(content)) {
    missingHeaders.push(header);
  }
});

if (missingHeaders.length > 0) {
  console.error(`FAIL: Missing required section headers: ${missingHeaders.join(', ')}`);
  process.exit(1);
}

console.log('PASS: Chapter structure is valid.');
process.exit(0);
