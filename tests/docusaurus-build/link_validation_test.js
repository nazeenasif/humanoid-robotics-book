/**
 * Automated test for internal navigation/link validation in the Humanoid Robotics Book
 * Verifies that all internal links are valid and navigation works correctly
 */

const fs = require('fs');
const path = require('path');
const matter = require('gray-matter'); // For parsing frontmatter

class LinkValidator {
  constructor() {
    this.results = {
      passed: [],
      failed: [],
      warnings: []
    };
    this.basePath = path.join(__dirname, '..', '..');
    this.allDocs = [];
    this.allLinks = [];
    this.validDocPaths = new Set();
  }

  /**
   * Run the link validation test
   */
  async runTest() {
    console.log('🔗 Running Link Validation Test...\n');

    try {
      // Collect all documentation files
      this.collectDocumentationFiles();

      // Parse all documents and extract links
      this.parseDocuments();

      // Validate all internal links
      this.validateInternalLinks();

      // Check for orphaned documents (not linked from anywhere)
      this.checkOrphanedDocuments();

      // Print results
      this.printResults();

      return this.results.failed.length === 0;
    } catch (error) {
      console.error('❌ Error during link validation:', error.message);
      this.results.failed.push(`Link validation error: ${error.message}`);
      return false;
    }
  }

  /**
   * Collect all documentation files
   */
  collectDocumentationFiles() {
    console.log('📂 Collecting documentation files...\n');

    const docsPath = path.join(this.basePath, 'docs');
    this.collectFilesRecursively(docsPath, '.md');
    this.collectFilesRecursively(docsPath, '.mdx');

    console.log(`  Found ${this.allDocs.length} documentation files\n`);
  }

  /**
   * Recursively collect files with specific extension
   */
  collectFilesRecursively(dir, extension) {
    const items = fs.readdirSync(dir);

    for (const item of items) {
      const itemPath = path.join(dir, item);
      const stat = fs.statSync(itemPath);

      if (stat.isDirectory()) {
        this.collectFilesRecursively(itemPath, extension);
      } else if (path.extname(item) === extension) {
        const relativePath = path.relative(this.basePath, itemPath);
        this.allDocs.push({
          path: itemPath,
          relativePath: relativePath,
          id: this.convertPathToId(relativePath)
        });
        this.validDocPaths.add(this.convertPathToId(relativePath));
      }
    }
  }

  /**
   * Convert file path to Docusaurus document ID
   */
  convertPathToId(filePath) {
    let id = filePath
      .replace('docs/', '')
      .replace(/\\/g, '/') // Normalize path separators
      .replace(/\.mdx?$/, ''); // Remove extension

    // Handle index files
    if (id.endsWith('/index')) {
      id = id.replace(/\/index$/, '/');
    } else if (id.endsWith('/index/index')) {
      id = id.replace(/\/index\/index$/, '/');
    }

    return id;
  }

  /**
   * Parse all documents and extract links
   */
  parseDocuments() {
    console.log('📄 Parsing documents and extracting links...\n');

    for (const doc of this.allDocs) {
      try {
        const content = fs.readFileSync(doc.path, 'utf8');
        const { data, content: body } = matter(content);

        // Extract all internal links from the document
        const links = this.extractLinks(body, doc.relativePath);

        // Add links to our collection
        for (const link of links) {
          this.allLinks.push({
            source: doc.id,
            target: link.target,
            raw: link.raw,
            type: link.type,
            line: link.line
          });
        }

        // Validate frontmatter for common fields
        this.validateFrontmatter(data, doc);

        console.log(`    ✅ Parsed: ${doc.relativePath}`);
      } catch (error) {
        const errorMsg = `Error parsing ${doc.relativePath}: ${error.message}`;
        this.results.failed.push(errorMsg);
        console.log(`    ❌ ${doc.relativePath} - Error: ${error.message}`);
      }
    }
    console.log(''); // Empty line for readability
  }

  /**
   * Extract internal links from document content
   */
  extractLinks(content, sourcePath) {
    const links = [];
    const lines = content.split('\n');

    // Regular expression to match markdown links and Docusaurus admonitions
    const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
    const relativeLinkRegex = /\{@import ([^}]+)\}/g; // For Docusaurus imports

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      let match;

      // Extract markdown links
      while ((match = linkRegex.exec(line)) !== null) {
        const fullMatch = match[0];
        const linkTarget = match[2];

        // Check if it's an internal link (starts with / or doesn't have protocol)
        if (this.isInternalLink(linkTarget)) {
          links.push({
            target: this.normalizeLink(linkTarget, sourcePath),
            raw: fullMatch,
            type: 'markdown',
            line: i + 1
          });
        }
      }

      // Extract relative imports
      while ((match = relativeLinkRegex.exec(line)) !== null) {
        const fullMatch = match[0];
        const linkTarget = match[1];

        if (this.isInternalLink(linkTarget)) {
          links.push({
            target: this.normalizeLink(linkTarget, sourcePath),
            raw: fullMatch,
            type: 'import',
            line: i + 1
          });
        }
      }
    }

    return links;
  }

  /**
   * Check if a link is internal
   */
  isInternalLink(link) {
    return !link.startsWith('http://') &&
           !link.startsWith('https://') &&
           !link.startsWith('mailto:') &&
           !link.startsWith('ftp://');
  }

  /**
   * Normalize link relative to source document
   */
  normalizeLink(link, sourcePath) {
    // Remove leading slash if present
    let normalized = link.startsWith('/') ? link.substring(1) : link;

    // If it's a relative link, resolve it relative to source
    if (normalized.startsWith('./') || normalized.startsWith('../')) {
      const sourceDir = path.dirname(sourcePath);
      normalized = path.posix.join(sourceDir, normalized).replace(/\\/g, '/');
    }

    // Remove .md or .mdx extensions
    normalized = normalized.replace(/\.mdx?$/, '');

    // Handle special Docusaurus patterns
    if (normalized.includes('#')) {
      const [pathPart] = normalized.split('#');
      normalized = pathPart;
    }

    return normalized;
  }

  /**
   * Validate document frontmatter
   */
  validateFrontmatter(frontmatter, doc) {
    // Check for required fields
    const requiredFields = ['title'];
    for (const field of requiredFields) {
      if (!frontmatter[field]) {
        const warnMsg = `Missing required frontmatter field '${field}' in ${doc.relativePath}`;
        this.results.warnings.push(warnMsg);
      }
    }

    // Check for common fields that should be strings
    const stringFields = ['title', 'description'];
    for (const field of stringFields) {
      if (frontmatter[field] && typeof frontmatter[field] !== 'string') {
        const warnMsg = `Frontmatter field '${field}' should be a string in ${doc.relativePath}`;
        this.results.warnings.push(warnMsg);
      }
    }
  }

  /**
   * Validate all internal links
   */
  validateInternalLinks() {
    console.log('🔍 Validating internal links...\n');

    let validLinks = 0;
    let invalidLinks = 0;

    for (const link of this.allLinks) {
      if (this.validDocPaths.has(link.target) || this.validDocPaths.has(link.target + '/index')) {
        this.results.passed.push(`Valid link: ${link.source} -> ${link.target}`);
        validLinks++;
        console.log(`    ✅ ${link.source} -> ${link.target}`);
      } else {
        const errorMsg = `Invalid internal link: ${link.source} -> ${link.target} (from ${link.source}, line ${link.line})`;
        this.results.failed.push(errorMsg);
        invalidLinks++;
        console.log(`    ❌ ${link.source} -> ${link.target}`);
      }
    }

    console.log(`\n  Valid links: ${validLinks}`);
    console.log(`  Invalid links: ${invalidLinks}\n`);
  }

  /**
   * Check for orphaned documents (not linked from anywhere)
   */
  checkOrphanedDocuments() {
    console.log('🔍 Checking for orphaned documents...\n');

    // Find documents that are not targets of any internal link
    const linkedDocs = new Set();
    for (const link of this.allLinks) {
      if (this.validDocPaths.has(link.target)) {
        linkedDocs.add(link.target);
      }
      // Also add the index version if it exists
      if (this.validDocPaths.has(link.target + '/index')) {
        linkedDocs.add(link.target + '/index');
      }
    }

    let orphanedCount = 0;
    for (const doc of this.allDocs) {
      // Skip index files and special files that might not need linking
      if (!doc.id.includes('index') &&
          !doc.id.endsWith('/index') &&
          !doc.id.includes('_category_') &&
          !linkedDocs.has(doc.id)) {
        const warnMsg = `Orphaned document: ${doc.id} (not linked from any other document)`;
        this.results.warnings.push(warnMsg);
        console.log(`    ⚠️  ${doc.id} - Not linked from other docs`);
        orphanedCount++;
      }
    }

    console.log(`\n  Orphaned documents: ${orphanedCount}\n`);
  }

  /**
   * Print test results
   */
  printResults() {
    console.log('='.repeat(60));
    console.log('LINK VALIDATION TEST RESULTS');
    console.log('='.repeat(60));

    const totalPassed = this.results.passed.length;
    const totalFailed = this.results.failed.length;
    const totalWarnings = this.results.warnings.length;
    const totalTests = totalPassed + totalFailed;

    console.log(`Total checks: ${totalTests}`);
    console.log(`Passed: ${totalPassed}`);
    console.log(`Failed: ${totalFailed}`);
    console.log(`Warnings: ${totalWarnings}\n`);

    if (this.results.failed.length > 0) {
      console.log('❌ FAILED CHECKS (Invalid Links):');
      this.results.failed.slice(0, 10).forEach(failure => {
        console.log(`  • ${failure}`);
      });
      if (this.results.failed.length > 10) {
        console.log(`  ... and ${this.results.failed.length - 10} more`);
      }
      console.log('');
    }

    if (this.results.warnings.length > 0) {
      console.log('⚠️  WARNINGS:');
      this.results.warnings.slice(0, 10).forEach(warning => {
        console.log(`  • ${warning}`);
      });
      if (this.results.warnings.length > 10) {
        console.log(`  ... and ${this.results.warnings.length - 10} more`);
      }
      console.log('');
    }

    if (this.results.passed.length > 0) {
      console.log('✅ PASSED CHECKS (Valid Links - showing first 10):');
      this.results.passed.slice(0, 10).forEach(pass => {
        console.log(`  • ${pass}`);
      });
      if (this.results.passed.length > 10) {
        console.log(`  ... and ${this.results.passed.length - 10} more valid links`);
      }
      console.log('');
    }

    const successRate = totalTests > 0 ? (totalPassed / totalTests) * 100 : 100;
    console.log(`Success Rate: ${successRate.toFixed(1)}%`);

    if (totalFailed === 0) {
      console.log('\n🎉 All links are valid!');
      process.exit(0);
    } else {
      console.log(`\n❌ ${totalFailed} links are invalid.`);
      process.exit(1);
    }
  }
}

// Run the test if this file is executed directly
if (require.main === module) {
  // Check if gray-matter is available, install if not
  try {
    require.resolve('gray-matter');
  } catch (e) {
    console.log('⚠️  gray-matter package not found. Installing...');
    const { execSync } = require('child_process');
    execSync('npm install gray-matter', { stdio: 'inherit' });
  }

  const validator = new LinkValidator();
  validator.runTest().then(success => {
    if (!success) {
      process.exit(1);
    }
  }).catch(error => {
    console.error('❌ Link validation failed:', error);
    process.exit(1);
  });
}

module.exports = LinkValidator;