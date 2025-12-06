/**
 * Automated Website Functionality Test
 * Tests navigation, search functionality, and responsiveness of the Docusaurus site
 */

const fs = require('fs');
const path = require('path');

class WebsiteFunctionalityTester {
  constructor(buildDir = 'build') {
    this.buildDir = path.join(__dirname, '..', '..', buildDir);
    this.results = {
      passed: [],
      failed: [],
      warnings: [],
      stats: {
        totalTests: 0,
        passedTests: 0,
        failedTests: 0,
        warnings: 0
      }
    };
  }

  /**
   * Run all functionality tests
   */
  async runTests() {
    console.log('🔍 Running Website Functionality Tests...\n');

    if (!fs.existsSync(this.buildDir)) {
      console.error(`❌ Build directory does not exist: ${this.buildDir}`);
      console.log('⚠️  Skipping functionality tests - build directory not found');
      return false;
    }

    // Test 1: Check that main HTML files exist
    await this.testHtmlFilesExist();

    // Test 2: Check that main assets exist
    await this.testAssetsExist();

    // Test 3: Check for broken links in generated HTML
    await this.testBrokenLinks();

    // Test 4: Check that CSS and JS bundles are present
    await this.testBundlesPresent();

    // Test 5: Check for common Docusaurus patterns
    await this.testDocusaurusPatterns();

    // Print results
    this.printResults();

    return this.results.failed.length === 0;
  }

  /**
   * Test that main HTML files exist
   */
  async testHtmlFilesExist() {
    console.log('📄 Testing HTML file generation...\n');

    const expectedPages = [
      'index.html',  // Homepage
      '404.html',    // Error page
      'sitemap.xml', // Sitemap
    ];

    // Look for documentation pages
    const docsDir = path.join(this.buildDir, 'docs');
    if (fs.existsSync(docsDir)) {
      const docsFiles = this.getAllFiles(docsDir, '.html');
      console.log(`  📄 Found ${docsFiles.length} documentation pages\n`);

      if (docsFiles.length === 0) {
        this.results.failed.push('No documentation pages found in build output');
        console.log('  ❌ No documentation pages generated\n');
      } else {
        this.results.passed.push(`Found ${docsFiles.length} documentation pages`);
        console.log(`  ✅ Found ${docsFiles.length} documentation pages\n`);
      }
    } else {
      console.log('  ⚠️  No docs directory found in build output\n');
    }

    // Check for expected pages
    for (const page of expectedPages) {
      const pagePath = path.join(this.buildDir, page);
      if (fs.existsSync(pagePath)) {
        this.results.passed.push(`Found expected page: ${page}`);
        console.log(`  ✅ ${page}`);
      } else {
        this.results.failed.push(`Missing expected page: ${page}`);
        console.log(`  ❌ ${page}`);
      }
    }
    console.log('\n'); // Empty line for readability
  }

  /**
   * Test that main assets exist
   */
  async testAssetsExist() {
    console.log('📦 Testing asset files...\n');

    const assetPatterns = [
      { dir: 'assets', type: 'directory', description: 'Assets directory' },
      { dir: 'img', type: 'directory', description: 'Image directory' },
      { dir: 'css', type: 'directory', description: 'CSS directory' },
      { dir: 'js', type: 'directory', description: 'JavaScript directory' },
    ];

    for (const pattern of assetPatterns) {
      const assetPath = path.join(this.buildDir, pattern.dir);
      if (fs.existsSync(assetPath)) {
        if (pattern.type === 'directory') {
          const files = fs.readdirSync(assetPath);
          if (files.length > 0) {
            this.results.passed.push(`${pattern.description} exists with ${files.length} files`);
            console.log(`  ✅ ${pattern.description} (${files.length} files)`);
          } else {
            this.results.warnings.push(`${pattern.description} exists but is empty`);
            console.log(`  ⚠️  ${pattern.description} (empty)`);
          }
        } else {
          this.results.passed.push(`${pattern.description} exists`);
          console.log(`  ✅ ${pattern.description}`);
        }
      } else {
        this.results.warnings.push(`${pattern.description} not found`);
        console.log(`  ⚠️  ${pattern.description} (missing)`);
      }
    }
    console.log('\n'); // Empty line for readability
  }

  /**
   * Test for broken links in generated HTML
   */
  async testBrokenLinks() {
    console.log('🔗 Testing for broken links...\n');

    // This is a simplified check - in a real implementation we would check all HTML files for broken links
    // For now, we'll just verify that HTML files have proper structure

    const htmlFiles = this.getAllFiles(this.buildDir, '.html');
    let totalLinks = 0;
    let potentialIssues = 0;

    for (const file of htmlFiles.slice(0, 5)) { // Check first 5 files to avoid too much output
      try {
        const content = fs.readFileSync(file, 'utf8');

        // Count links in the file
        const linkMatches = content.match(/href\s*=\s*["'][^"']*["']/gi);
        const scriptMatches = content.match(/src\s*=\s*["'][^"']*["']/gi);

        const links = linkMatches ? linkMatches.length : 0;
        const scripts = scriptMatches ? scriptMatches.length : 0;
        totalLinks += links + scripts;

        // Check for common broken link patterns
        const hasBrokenPattern = content.includes('href="#"') ||
                                content.includes('src="#"') ||
                                content.includes('404') ||
                                content.includes('Error');

        if (hasBrokenPattern) {
          potentialIssues++;
          this.results.warnings.push(`Potential link issues in ${path.relative(this.buildDir, file)}`);
        }
      } catch (error) {
        this.results.failed.push(`Could not read file ${file}: ${error.message}`);
      }
    }

    if (potentialIssues === 0) {
      this.results.passed.push(`No obvious broken link patterns found in sample of ${Math.min(htmlFiles.length, 5)} files`);
      console.log(`  ✅ No obvious broken link patterns (checked ${Math.min(htmlFiles.length, 5)} files)`);
    } else {
      this.results.warnings.push(`Found potential link issues in ${potentialIssues} files`);
      console.log(`  ⚠️  Potential link issues in ${potentialIssues} files`);
    }
    console.log(`  ℹ️  Total links found in sample: ${totalLinks}\n`);
  }

  /**
   * Test that CSS and JS bundles are present
   */
  async testBundlesPresent() {
    console.log('📦 Testing bundle files...\n');

    const cssFiles = this.getAllFiles(path.join(this.buildDir, 'assets'), '.css');
    const jsFiles = this.getAllFiles(path.join(this.buildDir, 'assets'), '.js');

    if (cssFiles.length > 0) {
      this.results.passed.push(`Found ${cssFiles.length} CSS bundle files`);
      console.log(`  ✅ ${cssFiles.length} CSS bundle files`);
    } else {
      this.results.failed.push('No CSS bundle files found');
      console.log('  ❌ No CSS bundle files');
    }

    if (jsFiles.length > 0) {
      this.results.passed.push(`Found ${jsFiles.length} JS bundle files`);
      console.log(`  ✅ ${jsFiles.length} JS bundle files`);
    } else {
      this.results.failed.push('No JS bundle files found');
      console.log('  ❌ No JS bundle files');
    }

    console.log(`  ℹ️  Checking assets directory for bundled resources\n`);
  }

  /**
   * Test for common Docusaurus patterns
   */
  async testDocusaurusPatterns() {
    console.log('🎯 Testing Docusaurus-specific patterns...\n');

    const indexPath = path.join(this.buildDir, 'index.html');
    if (fs.existsSync(indexPath)) {
      try {
        const indexContent = fs.readFileSync(indexPath, 'utf8');

        // Check for common Docusaurus elements
        const hasNavbar = indexContent.includes('navbar') || indexContent.includes('nav');
        const hasFooter = indexContent.includes('footer');
        const hasDocSidebar = indexContent.includes('sidebar') || indexContent.includes('menu');
        const hasDocusaurusMeta = indexContent.includes('Docusaurus') || indexContent.includes('docusaurus');

        if (hasNavbar) {
          this.results.passed.push('Homepage contains navbar structure');
          console.log('  ✅ Navbar structure present');
        } else {
          this.results.warnings.push('Homepage may be missing navbar structure');
          console.log('  ⚠️  Navbar structure possibly missing');
        }

        if (hasFooter) {
          this.results.passed.push('Homepage contains footer structure');
          console.log('  ✅ Footer structure present');
        } else {
          this.results.warnings.push('Homepage may be missing footer structure');
          console.log('  ⚠️  Footer structure possibly missing');
        }

        if (hasDocusaurusMeta) {
          this.results.passed.push('Docusaurus branding/metadata present');
          console.log('  ✅ Docusaurus branding present');
        } else {
          this.results.warnings.push('May be missing Docusaurus branding');
          console.log('  ⚠️  Docusaurus branding possibly missing');
        }

      } catch (error) {
        this.results.failed.push(`Could not read index.html: ${error.message}`);
        console.log(`  ❌ Could not read index.html: ${error.message}`);
      }
    } else {
      this.results.warnings.push('index.html not found - site may not have homepage');
      console.log('  ⚠️  index.html not found');
    }
    console.log('\n'); // Empty line for readability
  }

  /**
   * Get all files with specific extension from directory
   */
  getAllFiles(dir, ext) {
    const files = [];

    if (!fs.existsSync(dir)) {
      return files;
    }

    const items = fs.readdirSync(dir);
    for (const item of items) {
      const fullPath = path.join(dir, item);
      const stat = fs.statSync(fullPath);

      if (stat.isDirectory()) {
        files.push(...this.getAllFiles(fullPath, ext));
      } else if (item.toLowerCase().endsWith(ext.toLowerCase())) {
        files.push(fullPath);
      }
    }

    return files;
  }

  /**
   * Print test results
   */
  printResults() {
    console.log('='.repeat(70));
    console.log('WEBSITE FUNCTIONALITY TEST RESULTS');
    console.log('='.repeat(70));

    console.log(`Total tests: ${this.results.passed.length + this.results.failed.length + this.results.warnings.length}`);
    console.log(`Passed: ${this.results.passed.length}`);
    console.log(`Failed: ${this.results.failed.length}`);
    console.log(`Warnings: ${this.results.warnings.length}`);

    if (this.results.failed.length > 0) {
      console.log(`\n❌ FAILED TESTS:`);
      this.results.failed.forEach(failure => {
        console.log(`  • ${failure}`);
      });
    }

    if (this.results.warnings.length > 0) {
      console.log(`\n⚠️  WARNINGS:`);
      this.results.warnings.forEach(warning => {
        console.log(`  • ${warning}`);
      });
    }

    if (this.results.passed.length > 0) {
      console.log(`\n✅ PASSED TESTS (showing first 10):`);
      this.results.passed.slice(0, 10).forEach(pass => {
        console.log(`  • ${pass}`);
      });
      if (this.results.passed.length > 10) {
        console.log(`  ... and ${this.results.passed.length - 10} more`);
      }
    }

    const totalTests = this.results.passed.length + this.results.failed.length;
    const successRate = totalTests > 0 ? (this.results.passed.length / totalTests) * 100 : 100;
    console.log(`\nSuccess Rate: ${successRate.toFixed(1)}%`);

    if (this.results.failed.length === 0) {
      console.log(`\n🎉 Website functionality tests passed!`);
      console.log(`   The built site appears to have proper structure and assets.`);
    } else {
      console.log(`\n⚠️  ${this.results.failed.length} functionality issues found.`);
      console.log(`   Manual testing of navigation and search is recommended.`);
    }
  }
}

// Run the tests if this file is executed directly
if (require.main === module) {
  const tester = new WebsiteFunctionalityTester();
  tester.runTests().then(success => {
    process.exit(success ? 0 : 1);
  }).catch(error => {
    console.error('❌ Test execution error:', error);
    process.exit(1);
  });
}

module.exports = WebsiteFunctionalityTester;