/**
 * Automated test for structure completeness in the Humanoid Robotics Book
 * Verifies that all required modules, chapters, and assets exist
 */

const fs = require('fs');
const path = require('path');

// Define the expected structure
const expectedStructure = {
  modules: [
    {
      name: 'module1-ros2',
      requiredFiles: [
        'introduction.md',
        'python-integration.md',
        'humanoid-modeling.md',
        'exercises-ros2.md',
        'learning-outcomes.md',
        '_category_.json'
      ]
    },
    {
      name: 'module2-digital-twin',
      requiredFiles: [
        'introduction.md',
        'gazebo-simulation.md',
        'unity-simulation.md',
        'exercises-digital-twin.md',
        'learning-outcomes.md',
        '_category_.json'
      ]
    },
    {
      name: 'module3-ai-robot-brain',
      requiredFiles: [
        'introduction.md',
        'vslam.md',
        'nav2-ai-navigation.md',
        'reinforcement-learning.md',
        'exercises-ai-robot-brain.md',
        'learning-outcomes.md',
        '_category_.json'
      ]
    },
    {
      name: 'module4-vla',
      requiredFiles: [
        'introduction.md',
        'vision-processing.md',
        'language-processing.md',
        'action-execution.md',
        'exercises-vla.md',
        'capstone-project.md',
        'learning-outcomes.md',
        '_category_.json'
      ]
    }
  ],
  rootDocs: [
    'content-guidelines.md',
    'glossary.md'
  ],
  requiredDirectories: [
    'code-examples',
    'references',
    'tests'
  ],
  codeExampleDirectories: [
    'ros2',
    'gazebo',
    'unity',
    'isaac-sim',
    'vla'
  ]
};

class StructureCompletenessTester {
  constructor() {
    this.results = {
      passed: [],
      failed: [],
      warnings: []
    };
    this.basePath = path.join(__dirname, '..', '..');
  }

  /**
   * Run the structure completeness test
   */
  runTest() {
    console.log('🔍 Running Structure Completeness Test...\n');

    // Test module structures
    this.testModuleStructures();

    // Test root documentation files
    this.testRootDocs();

    // Test required directories
    this.testRequiredDirectories();

    // Test code examples structure
    this.testCodeExamples();

    // Print results
    this.printResults();

    return this.results.failed.length === 0;
  }

  /**
   * Test all module structures
   */
  testModuleStructures() {
    console.log('📂 Testing module structures...\n');

    for (const module of expectedStructure.modules) {
      const modulePath = path.join(this.basePath, 'docs', module.name);

      console.log(`  📁 Testing ${module.name}...`);

      // Check if module directory exists
      if (!fs.existsSync(modulePath)) {
        const errorMsg = `Module directory missing: ${modulePath}`;
        this.results.failed.push(errorMsg);
        console.log(`    ❌ ${errorMsg}`);
        continue;
      }

      // Check each required file
      for (const file of module.requiredFiles) {
        const filePath = path.join(modulePath, file);
        if (fs.existsSync(filePath)) {
          this.results.passed.push(`Found: docs/${module.name}/${file}`);
          console.log(`    ✅ ${file}`);
        } else {
          const errorMsg = `Missing file: docs/${module.name}/${file}`;
          this.results.failed.push(errorMsg);
          console.log(`    ❌ ${file}`);
        }
      }
      console.log(''); // Empty line for readability
    }
  }

  /**
   * Test root documentation files
   */
  testRootDocs() {
    console.log('📄 Testing root documentation files...\n');

    for (const file of expectedStructure.rootDocs) {
      const filePath = path.join(this.basePath, 'docs', file);

      if (fs.existsSync(filePath)) {
        this.results.passed.push(`Found: docs/${file}`);
        console.log(`  ✅ docs/${file}`);
      } else {
        // For glossary.md, it's optional but we'll warn if missing
        if (file === 'glossary.md') {
          const warnMsg = `Optional file missing: docs/${file} (consider creating for better user experience)`;
          this.results.warnings.push(warnMsg);
          console.log(`  ⚠️  ${file} - Optional but recommended`);
        } else {
          const errorMsg = `Missing required file: docs/${file}`;
          this.results.failed.push(errorMsg);
          console.log(`  ❌ ${file}`);
        }
      }
    }
    console.log(''); // Empty line for readability
  }

  /**
   * Test required directories
   */
  testRequiredDirectories() {
    console.log('📁 Testing required directories...\n');

    for (const dir of expectedStructure.requiredDirectories) {
      const dirPath = path.join(this.basePath, dir);

      if (fs.existsSync(dirPath) && fs.lstatSync(dirPath).isDirectory()) {
        this.results.passed.push(`Found directory: ${dir}/`);
        console.log(`  ✅ ${dir}/`);
      } else {
        const errorMsg = `Missing required directory: ${dir}/`;
        this.results.failed.push(errorMsg);
        console.log(`  ❌ ${dir}/`);
      }
    }
    console.log(''); // Empty line for readability
  }

  /**
   * Test code examples structure
   */
  testCodeExamples() {
    console.log('⚙️  Testing code examples structure...\n');

    const codeExamplesPath = path.join(this.basePath, 'code-examples');

    if (!fs.existsSync(codeExamplesPath)) {
      const errorMsg = `Missing code-examples directory`;
      this.results.failed.push(errorMsg);
      console.log(`  ❌ code-examples/ directory`);
      return;
    }

    for (const dir of expectedStructure.codeExampleDirectories) {
      const dirPath = path.join(codeExamplesPath, dir);

      if (fs.existsSync(dirPath) && fs.lstatSync(dirPath).isDirectory()) {
        this.results.passed.push(`Found code example directory: code-examples/${dir}/`);
        console.log(`  ✅ code-examples/${dir}/`);
      } else {
        const errorMsg = `Missing code example directory: code-examples/${dir}/`;
        this.results.failed.push(errorMsg);
        console.log(`  ❌ code-examples/${dir}/`);
      }
    }
    console.log(''); // Empty line for readability
  }

  /**
   * Print test results
   */
  printResults() {
    console.log('='.repeat(60));
    console.log('STRUCTURE COMPLETENESS TEST RESULTS');
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
      console.log('❌ FAILED CHECKS:');
      this.results.failed.forEach(failure => {
        console.log(`  • ${failure}`);
      });
      console.log('');
    }

    if (this.results.warnings.length > 0) {
      console.log('⚠️  WARNINGS:');
      this.results.warnings.forEach(warning => {
        console.log(`  • ${warning}`);
      });
      console.log('');
    }

    if (this.results.passed.length > 0) {
      console.log('✅ PASSED CHECKS (showing first 10):');
      const passedToShow = this.results.passed.slice(0, 10);
      passedToShow.forEach(pass => {
        console.log(`  • ${pass}`);
      });
      if (this.results.passed.length > 10) {
        console.log(`  ... and ${this.results.passed.length - 10} more`);
      }
      console.log('');
    }

    const successRate = totalTests > 0 ? (totalPassed / totalTests) * 100 : 100;
    console.log(`Success Rate: ${successRate.toFixed(1)}%`);

    if (totalFailed === 0) {
      console.log('\n🎉 All structure checks passed!');
      process.exit(0);
    } else {
      console.log(`\n❌ ${totalFailed} structure checks failed.`);
      process.exit(1);
    }
  }
}

// Run the test if this file is executed directly
if (require.main === module) {
  const tester = new StructureCompletenessTester();
  const allPassed = tester.runTest();
}

module.exports = StructureCompletenessTester;