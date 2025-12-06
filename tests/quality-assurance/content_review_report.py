#!/usr/bin/env python3
"""
Content Review Report for Humanoid Robotics Book

This script reviews all chapters for content accuracy, clarity, and adherence to target audience level.
"""

import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple, Any
import yaml
from collections import Counter

class ContentReviewReporter:
    def __init__(self, docs_dir: str = "docs", target_audience_level: str = "intermediate"):
        self.docs_dir = Path(docs_dir)
        self.target_audience_level = target_audience_level
        self.results = {
            "passed": [],
            "failed": [],
            "warnings": [],
            "stats": {
                "total_files": 0,
                "reviewed_files": 0,
                "accuracy_issues": 0,
                "clarity_issues": 0,
                "audience_issues": 0
            },
            "detailed_reviews": {}
        }

        # Define complexity indicators for target audience assessment
        self.complexity_indicators = {
            "advanced": {
                "terms": [
                    "tensor", "covariance", "eigenvector", "laplacian",
                    "homology", "manifold", "differentiable", "convolution",
                    "quaternion", "jacobian", "hessian", "optimization",
                    "stochastic", "bayesian", "monte carlo", "kalman filter",
                    "reinforcement learning", "deep learning", "neural network"
                ],
                "math_patterns": [
                    r'\d+\s*[+\-*/^]\s*\d+',  # Basic math
                    r'[a-zA-Z]\s*[+\-*/^]\s*[a-zA-Z]',  # Variable math
                    r'∫|∑|∏|∂|∇|∞|α|β|γ|δ|θ|λ|μ|π|σ|φ|ω',  # Math symbols
                    r'\d+\.\d+',  # Decimal numbers
                ]
            },
            "intermediate": {
                "terms": [
                    "algorithm", "function", "variable", "loop",
                    "conditional", "array", "matrix", "vector",
                    "probability", "statistics", "regression",
                    "classification", "clustering", "gradient"
                ]
            },
            "beginner": {
                "terms": [
                    "step", "example", "simple", "basic", "first",
                    "introduction", "overview", "concept", "idea"
                ]
            }
        }

        # Define accuracy check patterns
        self.accuracy_patterns = {
            "code_syntax": r'```(\w+)\n.*?\n```',
            "math_syntax": r'\$\$(.*?)\$\$|\$(.*?)\$',
            "urls": r'https?://[^\s<>"{}|\\^`\[\]]+',
        }

        # Define clarity metrics
        self.clarity_metrics = {
            "max_sentence_length": 25,  # words
            "max_paragraph_length": 10,  # sentences
            "min_explanation_ratio": 0.3,  # ratio of explanatory text to technical terms
        }

    def run_review(self) -> bool:
        """Run the content review."""
        print("Running Content Review...\n")

        # Check if docs directory exists
        if not self.docs_dir.exists():
            error_msg = f"Docs directory does not exist: {self.docs_dir}"
            self.results["failed"].append(error_msg)
            print(f"❌ {error_msg}")
            return False

        # Collect all markdown files
        md_files = list(self.docs_dir.rglob("*.md"))

        if not md_files:
            warn_msg = f"No markdown files found in {self.docs_dir}"
            self.results["warnings"].append(warn_msg)
            print(f"[WARN] {warn_msg}")
            return True

        print(f"Found {len(md_files)} markdown files to analyze\n")

        # Process each markdown file
        for md_file in md_files:
            if "glossary.md" in str(md_file) or "README.md" in str(md_file):  # Skip special files
                continue
            print(f"  Reviewing {md_file.relative_to(self.docs_dir)}...")
            self.review_markdown_file(md_file)
            print(f"    Completed review of {md_file.name}\n")

        # Generate comprehensive report
        self.generate_report()

        # Print results
        self.print_summary()

        # Return True if no critical failures
        return len(self.results["failed"]) == 0

    def review_markdown_file(self, file_path: Path):
        """Review a single markdown file for content quality."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            self.results["stats"]["total_files"] += 1

            # Parse frontmatter if present
            frontmatter = {}
            content_without_frontmatter = content
            if content.startswith('---'):
                lines = content.split('\n')
                frontmatter_end = -1
                for i, line in enumerate(lines[1:], 1):  # Skip first '---'
                    if line.strip() == '---':
                        frontmatter_end = i
                        break

                if frontmatter_end != -1:
                    frontmatter_content = '\n'.join(lines[1:frontmatter_end])
                    try:
                        frontmatter = yaml.safe_load(frontmatter_content)
                    except yaml.YAMLError:
                        pass
                    content_without_frontmatter = '\n'.join(lines[frontmatter_end+1:])

            # Perform various reviews
            accuracy_issues = self.check_accuracy(content_without_frontmatter, file_path)
            clarity_issues = self.check_clarity(content_without_frontmatter, file_path)
            audience_issues = self.check_audience_alignment(content_without_frontmatter, file_path)

            # Store detailed review
            self.results["detailed_reviews"][str(file_path)] = {
                "accuracy_issues": accuracy_issues,
                "clarity_issues": clarity_issues,
                "audience_issues": audience_issues,
                "frontmatter": frontmatter
            }

            # Update statistics
            self.results["stats"]["accuracy_issues"] += len(accuracy_issues)
            self.results["stats"]["clarity_issues"] += len(clarity_issues)
            self.results["stats"]["audience_issues"] += len(audience_issues)

            if accuracy_issues or clarity_issues or audience_issues:
                issue_count = len(accuracy_issues) + len(clarity_issues) + len(audience_issues)
                self.results["warnings"].append(f"{file_path.name}: {issue_count} issues found")
            else:
                self.results["passed"].append(f"Content review passed: {file_path.name}")

            self.results["stats"]["reviewed_files"] += 1

        except Exception as e:
            error_msg = f"Error reviewing file {file_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"    [ERROR] Error: {str(e)}")

    def check_accuracy(self, content: str, file_path: Path) -> List[str]:
        """Check content for accuracy issues."""
        issues = []

        # Check for incomplete code blocks
        code_block_matches = re.finditer(self.accuracy_patterns["code_syntax"], content, re.MULTILINE | re.DOTALL)
        for match in code_block_matches:
            full_match = match.group(0)
            lang = match.group(1) if match.group(1) else ""
            # Extract code content (everything between ``` markers)
            lines = full_match.split('\n')
            if len(lines) > 2:  # Has language spec and content
                code = '\n'.join(lines[1:-1])  # Remove opening and closing ```
            else:
                code = ""

            if not lang or lang.strip() == "":
                issues.append(f"Missing language specification in code block")
            if "TODO" in code or "FIXME" in code or "XXX" in code:
                issues.append(f"Unresolved placeholder in code block: {code[:50]}...")

        # Check for broken URLs
        urls = re.findall(self.accuracy_patterns["urls"], content)
        for url in urls:
            # In a real implementation, we might check if URLs are valid
            if "placeholder" in url.lower() or "example.com" in url:
                issues.append(f"Placeholder URL found: {url}")

        # Check for undefined terms in code comments
        lines = content.split('\n')
        for i, line in enumerate(lines, 1):
            if '//' in line or '#' in line or '/*' in line:
                if re.search(r'undefined|UNDEFINED', line, re.IGNORECASE):
                    issues.append(f"Undefined reference in comment at line {i}: {line.strip()}")

        return issues

    def check_clarity(self, content: str, file_path: Path) -> List[str]:
        """Check content for clarity issues."""
        issues = []

        # Split content into paragraphs
        paragraphs = re.split(r'\n\s*\n', content)

        for i, paragraph in enumerate(paragraphs):
            sentences = re.split(r'[.!?]+', paragraph)
            sentences = [s.strip() for s in sentences if s.strip()]

            # Check paragraph length
            if len(sentences) > self.clarity_metrics["max_paragraph_length"]:
                issues.append(f"Paragraph {i+1} is too long ({len(sentences)} sentences, max {self.clarity_metrics['max_paragraph_length']})")

            # Check each sentence
            for j, sentence in enumerate(sentences):
                words = sentence.split()

                # Check sentence length
                if len(words) > self.clarity_metrics["max_sentence_length"]:
                    issues.append(f"Sentence {j+1} in paragraph {i+1} is too long ({len(words)} words, max {self.clarity_metrics['max_sentence_length']})")

        # Check for technical jargon density
        words = re.findall(r'\b\w+\b', content.lower())
        technical_terms = [
            'algorithm', 'function', 'parameter', 'variable', 'method', 'class',
            'interface', 'protocol', 'framework', 'library', 'api', 'sdk'
        ]

        technical_count = sum(1 for word in words if word in technical_terms)
        if len(words) > 0:
            jargon_ratio = technical_count / len(words)
            if jargon_ratio > 0.3:  # More than 30% technical terms
                issues.append(f"High technical jargon density: {jargon_ratio:.1%} of words are technical terms")

        return issues

    def check_audience_alignment(self, content: str, file_path: Path) -> List[str]:
        """Check if content aligns with target audience."""
        issues = []

        # Count complexity indicators
        advanced_terms = 0
        intermediate_terms = 0
        beginner_terms = 0

        for term in self.complexity_indicators["advanced"]["terms"]:
            advanced_terms += len(re.findall(r'\b' + re.escape(term) + r'\b', content, re.IGNORECASE))

        for term in self.complexity_indicators["intermediate"]["terms"]:
            intermediate_terms += len(re.findall(r'\b' + re.escape(term) + r'\b', content, re.IGNORECASE))

        for term in self.complexity_indicators["beginner"]["terms"]:
            beginner_terms += len(re.findall(r'\b' + re.escape(term) + r'\b', content, re.IGNORECASE))

        # Check math complexity
        math_complexity = 0
        for pattern in self.complexity_indicators["advanced"]["math_patterns"]:
            math_complexity += len(re.findall(pattern, content))

        # Determine if content level matches target
        if self.target_audience_level == "intermediate":
            if advanced_terms > 10 and beginner_terms < 5:
                issues.append(f"Content appears too advanced for intermediate audience (advanced terms: {advanced_terms}, beginner terms: {beginner_terms})")
            elif beginner_terms > 15 and advanced_terms < 3:
                issues.append(f"Content appears too basic for intermediate audience (beginner terms: {beginner_terms}, advanced terms: {advanced_terms})")

        return issues

    def generate_report(self):
        """Generate a detailed content review report."""
        report_path = Path("tests/quality-assurance/content_review_report.md")

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("# Content Review Report\n\n")
            f.write(f"Date: {self.get_current_date()}\n\n")

            # Executive Summary
            f.write("## Executive Summary\n\n")
            stats = self.results["stats"]
            f.write(f"- Total files reviewed: {stats['reviewed_files']}\n")
            f.write(f"- Accuracy issues found: {stats['accuracy_issues']}\n")
            f.write(f"- Clarity issues found: {stats['clarity_issues']}\n")
            f.write(f"- Audience alignment issues: {stats['audience_issues']}\n\n")

            # Detailed Reviews
            f.write("## Detailed Reviews\n\n")
            for file_path, review in self.results["detailed_reviews"].items():
                f.write(f"### {Path(file_path).name}\n\n")

                if review["frontmatter"]:
                    f.write("**Frontmatter:**\n")
                    for key, value in review["frontmatter"].items():
                        f.write(f"- {key}: {value}\n")
                    f.write("\n")

                if review["accuracy_issues"]:
                    f.write("**Accuracy Issues:**\n")
                    for issue in review["accuracy_issues"]:
                        f.write(f"- {issue}\n")
                    f.write("\n")

                if review["clarity_issues"]:
                    f.write("**Clarity Issues:**\n")
                    for issue in review["clarity_issues"]:
                        f.write(f"- {issue}\n")
                    f.write("\n")

                if review["audience_issues"]:
                    f.write("**Audience Alignment Issues:**\n")
                    for issue in review["audience_issues"]:
                        f.write(f"- {issue}\n")
                    f.write("\n")

                f.write("---\n\n")

            # Recommendations
            f.write("## Recommendations\n\n")
            f.write("Based on the review, the following improvements are recommended:\n\n")
            f.write("1. **Accuracy**: Address all identified accuracy issues, particularly incomplete code examples and placeholder content.\n")
            f.write("2. **Clarity**: Reduce sentence and paragraph lengths to improve readability.\n")
            f.write("3. **Audience Alignment**: Ensure content complexity matches the target audience level (intermediate).\n")
            f.write("4. **Consistency**: Maintain consistent terminology and formatting across all chapters.\n")

    def get_current_date(self) -> str:
        """Get current date in YYYY-MM-DD format."""
        from datetime import datetime
        return datetime.now().strftime("%Y-%m-%d")

    def print_summary(self):
        """Print the review summary."""
        print("="*70)
        print("CONTENT REVIEW SUMMARY")
        print("="*70)

        stats = self.results["stats"]
        print(f"Total files: {stats['total_files']}")
        print(f"Files reviewed: {stats['reviewed_files']}")
        print(f"Accuracy issues: {stats['accuracy_issues']}")
        print(f"Clarity issues: {stats['clarity_issues']}")
        print(f"Audience issues: {stats['audience_issues']}")
        print(f"Total issues: {sum(stats.values()) - stats['total_files'] - stats['reviewed_files']}")

        print(f"\nPassed reviews: {len(self.results['passed'])}")
        print(f"Failed reviews: {len(self.results['failed'])}")
        print(f"Warnings: {len(self.results['warnings'])}")

        if self.results["failed"]:
            print(f"\n❌ FAILED REVIEWS (Critical Issues):")
            for failure in self.results["failed"]:
                print(f"  • {failure}")

        if self.results["warnings"]:
            print(f"\n[WARN] REVIEW ISSUES:")
            for warning in self.results["warnings"][:5]:  # Show first 5
                print(f"  • {warning}")
            if len(self.results["warnings"]) > 5:
                print(f"  ... and {len(self.results['warnings']) - 5} more issues")

        if self.results["passed"]:
            print(f"\n[OK] PASSED REVIEWS (showing first 5):")
            for passed in self.results["passed"][:5]:
                print(f"  • {passed}")
            if len(self.results["passed"]) > 5:
                print(f"  ... and {len(self.results['passed']) - 5} more")

        print(f"\n[DONE] Detailed report saved to: tests/quality-assurance/content_review_report.md")
        print(f"\n[INFO] {stats['reviewed_files']} files have been reviewed for accuracy, clarity, and audience alignment.")
        print(f"   Review report provides specific recommendations for improvements.")

def main():
    """Main function to run the content review."""
    print("Humanoid Robotics Book - Content Review Reporter")
    print("="*52)

    # Create tests directory if it doesn't exist
    tests_dir = Path("tests/quality-assurance")
    tests_dir.mkdir(parents=True, exist_ok=True)

    # Initialize and run reviewer
    reviewer = ContentReviewReporter()
    is_passing = reviewer.run_review()

    return is_passing

if __name__ == "__main__":
    main()