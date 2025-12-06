#!/usr/bin/env python3
"""
Automated test for APA citation validity in the Humanoid Robotics Book.

This script checks all documentation files to ensure citations follow proper APA format.
"""

import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import json

class APACitationValidator:
    def __init__(self, docs_dir: str = "docs", references_dir: str = "references"):
        self.docs_dir = Path(docs_dir)
        self.references_dir = Path(references_dir)
        self.results = {
            "passed": [],
            "failed": [],
            "warnings": [],
            "stats": {
                "total_citations": 0,
                "valid_citations": 0,
                "invalid_citations": 0,
                "citations_without_refs": 0
            }
        }

        # Common APA citation patterns
        self.apa_patterns = {
            # In-text citations: (Author, Year) or Author (Year)
            "parenthetical": r'\([A-Z][a-z]+,\s*\d{4}\)',
            "narrative": r'[A-Z][a-z]+\s*\(\d{4}\)',
            "multiple_authors": r'\([A-Z][a-z]+,\s*[A-Z][a-z]+,\s*&\s*[A-Z][a-z]+,\s*\d{4}\)',
            "et_al": r'\([A-Z][a-z]+,\s*et\s*al\.,\s*\d{4}\)',
            # Page numbers
            "with_pages": r'\([A-Z][a-z]+,\s*\d{4},\s*pp?\.\s*\d+-?\d*\)',
            # Multiple citations
            "multiple_citations": r'\([A-Z][a-z]+,\s*\d{4};\s*[A-Z][a-z]+,\s*\d{4}\)',
        }

        # Reference patterns for different types
        self.reference_patterns = {
            "journal": r"^[A-Z][A-Za-z\s\-]+\.?\s*\(\d{4}\)\.\s*[A-Z][A-Za-z\s\-\.,:;()]+\.\s*[A-Z][A-Za-z\s\-]+,\s*\d+\(\d+\),\s*\d+-\d+\.$",
            "book": r"^[A-Z][A-Za-z\s\-]+\.?\s*\(\d{4}\)\.\s*[A-Z][A-Za-z\s\-\.,:;()]+\.\s*[A-Z][A-Za-z\s\-]+:$",
            "conference": r"^[A-Z][A-Za-z\s\-]+\.?\s*\(\d{4}\)\.\s*[A-Z][A-Za-z\s\-\.,:;()]+\.\s*Proceedings of the.*",
            "web": r"^[A-Z][A-Za-z\s\-]+\.?\s*\(\d{4}\)\.\s*[A-Z][A-Za-z\s\-\.,:;()]+\.\s*Retrieved from\s*https?://.*$",
        }

        # Known valid reference entries
        self.valid_references = set()
        self.extract_valid_references()

    def extract_valid_references(self):
        """Extract valid reference entries from reference files."""
        if self.references_dir.exists():
            for ref_file in self.references_dir.glob("*.md"):
                with open(ref_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    # Extract potential reference entries
                    # This is a simplified approach - could be enhanced based on specific reference format
                    lines = content.split('\n')
                    for line in lines:
                        if re.match(r'^\s*[A-Z].*\d{4}.*$', line.strip()):
                            # Normalize the line for comparison
                            normalized = re.sub(r'\s+', ' ', line.strip().lower())
                            self.valid_references.add(normalized)

    def run_test(self) -> bool:
        """Run the APA citation validity test."""
        print("🔍 Running APA Citation Validity Test...\n")

        # Check if docs directory exists
        if not self.docs_dir.exists():
            error_msg = f"Docs directory does not exist: {self.docs_dir}"
            self.results["failed"].append(error_msg)
            print(f"❌ {error_msg}")
            return False

        # Collect all markdown files
        md_files = list(self.docs_dir.rglob("*.md")) + list(self.docs_dir.rglob("*.mdx"))

        if not md_files:
            warn_msg = f"No markdown files found in {self.docs_dir}"
            self.results["warnings"].append(warn_msg)
            print(f"⚠️  {warn_msg}")
            return True  # Not a failure, just no files to check

        print(f"📄 Found {len(md_files)} markdown files to analyze\n")

        # Process each markdown file
        for md_file in md_files:
            print(f"  Analyzing {md_file.relative_to(self.docs_dir)}...")
            self.analyze_markdown_file(md_file)
            print(f"    Processed {md_file.name}\n")

        # Print results
        self.print_results()

        # Return True if no critical failures
        return len(self.results["failed"]) == 0

    def analyze_markdown_file(self, file_path: Path):
        """Analyze a single markdown file for APA citations."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Find all potential APA citations
            citations = self.find_apa_citations(content)

            for citation, line_num in citations:
                self.results["stats"]["total_citations"] += 1

                # Validate the citation format
                is_valid = self.validate_citation_format(citation)

                if is_valid:
                    self.results["stats"]["valid_citations"] += 1
                    self.results["passed"].append(f"Valid APA citation in {file_path.name}:{line_num} - {citation}")
                    print(f"    ✅ {citation[:50]}...")
                else:
                    self.results["stats"]["invalid_citations"] += 1
                    error_msg = f"Invalid APA citation format in {file_path.name}:{line_num} - {citation}"
                    self.results["failed"].append(error_msg)
                    print(f"    ❌ {citation[:50]}...")

            # Also check for reference sections and validate those
            self.validate_reference_section(content, file_path)

        except Exception as e:
            error_msg = f"Error analyzing file {file_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"    ❌ Error: {str(e)}")

    def find_apa_citations(self, content: str) -> List[Tuple[str, int]]:
        """Find potential APA citations in content."""
        citations = []
        lines = content.split('\n')

        for i, line in enumerate(lines, 1):
            # Find parenthetical citations (Author, Year)
            parenthetical_matches = re.finditer(r'\([A-Z][A-Za-z,\s.&\d-]+,\s*\d{4}\)', line)
            for match in parenthetical_matches:
                citations.append((match.group(), i))

            # Find narrative citations Author (Year)
            narrative_matches = re.finditer(r'[A-Z][A-Za-z\-\s]+\s*\(\d{4}\)', line)
            for match in narrative_matches:
                citations.append((match.group(), i))

            # Find citations with page numbers
            page_matches = re.finditer(r'\([A-Z][A-Za-z,\s.&\d-]+,\s*\d{4},\s*pp?\.?\s*\d+-?\d*\)', line)
            for match in page_matches:
                citations.append((match.group(), i))

        return citations

    def validate_citation_format(self, citation: str) -> bool:
        """Validate if a citation follows APA format."""
        # Remove extra spaces
        citation = re.sub(r'\s+', ' ', citation.strip())

        # Check for common APA patterns
        patterns = [
            # (Author, Year)
            r'^\([A-Z][a-z]+,\s*\d{4}\)$',
            # (Author, Author, & Author, Year)
            r'^\([A-Z][a-z]+,\s*[A-Z][a-z]+,\s*&\s*[A-Z][a-z]+,\s*\d{4}\)$',
            # (Author et al., Year)
            r'^\([A-Z][a-z]+,\s*et\s*al\.,\s*\d{4}\)$',
            # (Author, Year, p. #) or (Author, Year, pp. #-#)
            r'^\([A-Z][a-z]+,\s*\d{4},\s*pp?\.?\s*\d+-?\d*\)$',
            # (Author, Year); (Author, Year) - multiple citations
            r'^\([A-Z][a-z]+,\s*\d{4}\);\s*\([A-Z][a-z]+,\s*\d{4}\)$',
        ]

        for pattern in patterns:
            if re.match(pattern, citation):
                return True

        # Check for narrative citations
        narrative_patterns = [
            # Author (Year)
            r'^[A-Z][a-z]+\s*\(\d{4}\)$',
            # Author et al. (Year)
            r'^[A-Z][a-z]+\s*et\s*al\.\s*\(\d{4}\)$',
        ]

        for pattern in narrative_patterns:
            if re.match(pattern, citation):
                return True

        return False

    def validate_reference_section(self, content: str, file_path: Path):
        """Validate reference sections in the content."""
        # Look for reference or bibliography sections
        if re.search(r'#\s*[Rr]eferences|#\s*[Bb]ibliography', content):
            # Extract the references section
            ref_section_match = re.search(r'#\s*[Rr]eferences.*?(\n#\s.|$)', content, re.DOTALL)
            if ref_section_match:
                ref_section = ref_section_match.group(0)

                # Check each line in the reference section
                lines = ref_section.split('\n')
                for i, line in enumerate(lines):
                    if line.strip() and not line.startswith('#') and not line.startswith(' ') and not line.startswith('\t'):
                        # This might be a reference entry
                        if not self.is_valid_reference_format(line.strip()):
                            warn_msg = f"Potentially invalid reference format in {file_path.name} line {i}: {line.strip()[:50]}..."
                            self.results["warnings"].append(warn_msg)

    def is_valid_reference_format(self, reference: str) -> bool:
        """Check if a reference follows a valid format."""
        if not reference.strip():
            return True  # Empty line is valid

        # Normalize spaces
        ref = re.sub(r'\s+', ' ', reference.strip())

        # Check against known valid references
        normalized_ref = re.sub(r'\s+', ' ', ref.lower())
        if normalized_ref in [r.lower() for r in self.valid_references]:
            return True

        # Check against reference patterns
        for pattern_name, pattern in self.reference_patterns.items():
            if re.match(pattern, ref):
                return True

        # Simple check: should start with capital letter and end with period
        if re.match(r'^[A-Z].*\.$', ref):
            return True

        return False

    def print_results(self):
        """Print the test results."""
        print("="*60)
        print("APA CITATION VALIDITY TEST RESULTS")
        print("="*60)

        stats = self.results["stats"]
        print(f"Total citations found: {stats['total_citations']}")
        print(f"Valid citations: {stats['valid_citations']}")
        print(f"Invalid citations: {stats['invalid_citations']}")
        print(f"Success rate: {stats['valid_citations']/max(stats['total_citations'], 1)*100:.1f}%")

        print(f"\nPassed checks: {len(self.results['passed'])}")
        print(f"Failed checks: {len(self.results['failed'])}")
        print(f"Warnings: {len(self.results['warnings'])}")

        if self.results["failed"]:
            print(f"\n❌ FAILED CHECKS (Invalid Citation Formats):")
            for failure in self.results["failed"][:10]:  # Show first 10
                print(f"  • {failure}")
            if len(self.results["failed"]) > 10:
                print(f"  ... and {len(self.results['failed']) - 10} more")

        if self.results["warnings"]:
            print(f"\n⚠️  WARNINGS:")
            for warning in self.results["warnings"][:10]:  # Show first 10
                print(f"  • {warning}")
            if len(self.results["warnings"]) > 10:
                print(f"  ... and {len(self.results['warnings']) - 10} more")

        if self.results["passed"]:
            print(f"\n✅ PASSED CHECKS (Valid Citations - showing first 5):")
            for passed in self.results["passed"][:5]:
                print(f"  • {passed}")
            if len(self.results["passed"]) > 5:
                print(f"  ... and {len(self.results['passed']) - 5} more")

        success_rate = stats['valid_citations'] / max(stats['total_citations'], 1) * 100
        print(f"\nCitation Format Success Rate: {success_rate:.1f}%")

        if stats['invalid_citations'] == 0:
            print(f"\n🎉 All citations follow APA format!")
            sys.exit(0)
        else:
            print(f"\n❌ {stats['invalid_citations']} citations have invalid APA format.")
            print(f"   Note: This check focuses on format validity. Content accuracy is separate.")
            sys.exit(1)

def main():
    """Main function to run the APA citation validator."""
    print("Humanoid Robotics Book - APA Citation Validator")
    print("="*48)

    # Create tests directory if it doesn't exist
    tests_dir = Path("tests/quality-assurance")
    tests_dir.mkdir(parents=True, exist_ok=True)

    # Initialize and run validator
    validator = APACitationValidator()
    is_passing = validator.run_test()

    return is_passing

if __name__ == "__main__":
    main()