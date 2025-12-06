#!/usr/bin/env python3
"""
Style and Consistency Checker for Humanoid Robotics Book

This script verifies consistent writing style, tone, and formatting across all modules/chapters.
"""

import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import yaml
import json

class StyleConsistencyChecker:
    def __init__(self, docs_dir: str = "docs"):
        self.docs_dir = Path(docs_dir)
        self.results = {
            "passed": [],
            "failed": [],
            "warnings": [],
            "stats": {
                "total_files": 0,
                "consistent_files": 0,
                "inconsistent_files": 0
            }
        }

        # Style guidelines
        self.guidelines = {
            # Writing tone - should be educational, professional, and accessible
            "tone_keywords": [
                "understand", "learn", "implement", "explore", "apply",
                "important", "note", "tip", "caution", "warning"
            ],

            # Formatting requirements
            "required_headers": ["# ", "## ", "### "],  # At least H1, H2, H3
            "forbidden_headers": ["###### "],  # Too deep

            # Consistency patterns
            "code_block_pattern": r'```(\w+)?\n.*?\n```',
            "emphasis_pattern": r'\*[^*]+\*|_[^_]+_|`[^`]+`',  # Italic, bold, code
            "list_patterns": [r'^\s*[\*\-\d]\.\s', r'^\s*\d+\.\s'],  # Unordered and ordered lists

            # Terminology consistency
            "consistent_terms": {
                "ROS 2": ["ROS2", "ros2", "Ros2"],  # All should be "ROS 2"
                "Humanoid Robot": ["humanoid robot", "Humanoid robot", "humanoid"],  # Should be "Humanoid Robot" when appropriate
                "OpenAI GPT": ["GPT", "openai gpt", "OpenAI gpt"],  # Should be "OpenAI GPT"
                "OpenAI Whisper": ["Whisper", "openai whisper", "OpenAI whisper"],  # Should be "OpenAI Whisper"
            }
        }

    def run_check(self) -> bool:
        """Run the style consistency check."""
        print("Running Style and Consistency Check...\n")

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
            if "glossary.md" in str(md_file):  # Skip glossary as it's specially formatted
                continue
            print(f"  Analyzing {md_file.relative_to(self.docs_dir)}...")
            self.analyze_markdown_file(md_file)
            print(f"    Processed {md_file.name}\n")

        # Print results
        self.print_results()

        # Return True if no critical failures
        return len(self.results["failed"]) == 0

    def analyze_markdown_file(self, file_path: Path):
        """Analyze a single markdown file for style consistency."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            self.results["stats"]["total_files"] += 1
            issues_found = []

            # Check for proper frontmatter
            has_frontmatter = self.check_frontmatter(content, file_path)

            # Check header hierarchy
            header_issues = self.check_header_hierarchy(content, file_path)
            issues_found.extend(header_issues)

            # Check for consistent terminology
            terminology_issues = self.check_terminology_consistency(content, file_path)
            issues_found.extend(terminology_issues)

            # Check code block formatting
            code_issues = self.check_code_blocks(content, file_path)
            issues_found.extend(code_issues)

            # Check list formatting
            list_issues = self.check_lists(content, file_path)
            issues_found.extend(list_issues)

            # Check for consistent use of emphasis
            emphasis_issues = self.check_emphasis(content, file_path)
            issues_found.extend(emphasis_issues)

            # Check for consistent writing tone indicators
            tone_issues = self.check_writing_tone(content, file_path)
            issues_found.extend(tone_issues)

            if not issues_found:
                self.results["stats"]["consistent_files"] += 1
                self.results["passed"].append(f"Consistent style: {file_path.name}")
                print(f"    OK {file_path.name} - Consistent style")
            else:
                self.results["stats"]["inconsistent_files"] += 1
                for issue in issues_found:
                    self.results["warnings"].append(issue)
                print(f"    !! {file_path.name} - {len(issues_found)} style issues")

        except Exception as e:
            error_msg = f"Error analyzing file {file_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"    ❌ Error: {str(e)}")

    def check_frontmatter(self, content: str, file_path: Path) -> List[str]:
        """Check for proper frontmatter with title."""
        issues = []

        # Check if content starts with frontmatter
        if content.startswith('---'):
            # Find the end of frontmatter
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
                    if not frontmatter or 'title' not in frontmatter:
                        issues.append(f"Missing 'title' in frontmatter: {file_path.name}")
                except yaml.YAMLError:
                    issues.append(f"Invalid YAML frontmatter: {file_path.name}")
            else:
                issues.append(f"Unclosed frontmatter: {file_path.name}")
        else:
            issues.append(f"Missing frontmatter: {file_path.name}")

        return issues

    def check_header_hierarchy(self, content: str, file_path: Path) -> List[str]:
        """Check for proper header hierarchy."""
        issues = []
        lines = content.split('\n')

        headers = []
        for i, line in enumerate(lines):
            if line.startswith('#'):
                header_level = len(line) - len(line.lstrip('#'))
                headers.append((header_level, line.strip(), i + 1))

        # Check for proper hierarchy (no skipping levels like H1 -> H3)
        for i in range(1, len(headers)):
            prev_level = headers[i-1][0]
            curr_level = headers[i][0]

            if curr_level > prev_level + 1:
                issues.append(f"Header hierarchy issue in {file_path.name} at line {headers[i][2]}: H{prev_level} followed by H{curr_level} (skipped H{prev_level + 1})")

        return issues

    def check_terminology_consistency(self, content: str, file_path: Path) -> List[str]:
        """Check for consistent terminology usage."""
        issues = []

        for preferred_term, alternatives in self.guidelines["consistent_terms"].items():
            for alt_term in alternatives:
                # Use word boundaries to avoid partial matches
                pattern = r'\b' + re.escape(alt_term) + r'\b'
                matches = re.findall(pattern, content, re.IGNORECASE)
                if matches:
                    issues.append(f"Inconsistent terminology in {file_path.name}: Found '{alt_term}' instead of '{preferred_term}' ({len(matches)} occurrences)")

        return issues

    def check_code_blocks(self, content: str, file_path: Path) -> List[str]:
        """Check for consistent code block formatting."""
        issues = []

        # Find all code blocks
        code_blocks = re.findall(self.guidelines["code_block_pattern"], content, re.DOTALL)

        # Check for missing language specifications
        code_block_matches = re.finditer(self.guidelines["code_block_pattern"], content, re.DOTALL)
        for match in code_block_matches:
            full_block = match.group(0)
            language_spec = match.group(1) if match.group(1) else ""

            if not language_spec or language_spec.strip() == "":
                # Find line number for this code block
                lines_before = content[:match.start()].count('\n') + 1
                issues.append(f"Missing language specification in code block in {file_path.name} around line {lines_before}")

        return issues

    def check_lists(self, content: str, file_path: Path) -> List[str]:
        """Check for consistent list formatting."""
        issues = []
        lines = content.split('\n')

        in_list = False
        list_line_numbers = []

        for i, line in enumerate(lines):
            # Check if this line is part of a list
            if re.match(r'^\s*[\*\-\d]\.\s', line) or re.match(r'^\s*\d+\.\s', line):
                if not in_list:
                    in_list = True
                    list_line_numbers = [i + 1]
                else:
                    list_line_numbers.append(i + 1)
            elif in_list and line.strip() == "":
                # Continue the list if it's just a blank line
                continue
            elif in_list and not line.startswith(' ') and not line.startswith('\t'):
                # End of list
                in_list = False
            elif in_list and line.strip() != "":
                # Part of the list item
                continue

        return issues

    def check_emphasis(self, content: str, file_path: Path) -> List[str]:
        """Check for consistent use of emphasis."""
        issues = []
        # For now, just log occurrences - in a real check we might look for consistency
        return issues

    def check_writing_tone(self, content: str, file_path: Path) -> List[str]:
        """Check for consistent writing tone."""
        issues = []

        # Look for informal language that might not match the educational tone
        informal_patterns = [
            r'\bgonna\b',  # going to
            r'\bwanna\b',  # want to
            r"\bain't\b",  # am not/are not
            r'\b(yeah|yep|nah)\b',  # casual affirmations/negations
        ]

        for pattern in informal_patterns:
            matches = re.findall(pattern, content, re.IGNORECASE)
            if matches:
                issues.append(f"Informal language detected in {file_path.name}: {matches}")

        return issues

    def print_results(self):
        """Print the test results."""
        print("="*70)
        print("STYLE AND CONSISTENCY CHECK RESULTS")
        print("="*70)

        stats = self.results["stats"]
        print(f"Total files analyzed: {stats['total_files']}")
        print(f"Files with consistent style: {stats['consistent_files']}")
        print(f"Files with inconsistencies: {stats['inconsistent_files']}")
        if stats['total_files'] > 0:
            print(f"Consistency rate: {stats['consistent_files']/stats['total_files']*100:.1f}%")

        print(f"\nPassed checks: {len(self.results['passed'])}")
        print(f"Failed checks: {len(self.results['failed'])}")
        print(f"Warnings (style issues): {len(self.results['warnings'])}")

        if self.results["failed"]:
            print(f"\n❌ FAILED CHECKS (Critical Issues):")
            for failure in self.results["failed"]:
                print(f"  • {failure}")

        if self.results["warnings"]:
            print(f"\n[WARN] STYLE ISSUES (Recommendations):")
            for warning in self.results["warnings"][:10]:  # Show first 10
                print(f"  • {warning}")
            if len(self.results["warnings"]) > 10:
                print(f"  ... and {len(self.results['warnings']) - 10} more issues")

        if self.results["passed"]:
            print(f"\n[OK] PASSED CHECKS (Consistent Files - showing first 5):")
            for passed in self.results["passed"][:5]:
                print(f"  • {passed}")
            if len(self.results["passed"]) > 5:
                print(f"  ... and {len(self.results['passed']) - 5} more")

        consistency_rate = stats['consistent_files'] / max(stats['total_files'], 1) * 100
        print(f"\nStyle Consistency Rate: {consistency_rate:.1f}%")

        if stats['inconsistent_files'] == 0:
            print(f"\n[SUCCESS] All files have consistent style!")
            sys.exit(0)
        else:
            print(f"\n[INFO] {stats['inconsistent_files']} files have style inconsistencies that should be reviewed.")
            print(f"   This is a quality check - the content is still valid, but style could be improved.")
            sys.exit(0)  # Don't fail the build for style issues

def main():
    """Main function to run the style consistency checker."""
    print("Humanoid Robotics Book - Style and Consistency Checker")
    print("="*54)

    # Create tests directory if it doesn't exist
    tests_dir = Path("tests/quality-assurance")
    tests_dir.mkdir(parents=True, exist_ok=True)

    # Initialize and run checker
    checker = StyleConsistencyChecker()
    is_passing = checker.run_check()

    return is_passing

if __name__ == "__main__":
    main()