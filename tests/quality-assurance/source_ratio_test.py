#!/usr/bin/env python3
"""
Automated test for peer-reviewed source ratio in the Humanoid Robotics Book.

This script checks the references directory to ensure that at least 50% of sources
are peer-reviewed as specified in the quality validation criteria.
"""

import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple
import bibtexparser
from urllib.parse import urlparse

class PeerReviewedSourceChecker:
    def __init__(self, references_dir: str = "references", required_ratio: float = 0.5):
        self.references_dir = Path(references_dir)
        self.required_ratio = required_ratio
        self.results = {
            "passed": [],
            "failed": [],
            "warnings": [],
            "stats": {
                "total_sources": 0,
                "peer_reviewed": 0,
                "non_peer_reviewed": 0,
                "ratio": 0.0
            }
        }
        # Known peer-reviewed venues and publisher patterns
        self.peer_reviewed_patterns = [
            # Conferences
            r'.*icra.*',  # International Conference on Robotics and Automation
            r'.*iros.*',  # International Conference on Intelligent Robots and Systems
            r'.*rss.*',   # Robotics: Science and Systems
            r'.*corl.*',  # Conference on Robot Learning
            r'.*cvpr.*',  # Computer Vision and Pattern Recognition
            r'.*iccv.*',  # International Conference on Computer Vision
            r'.*eccv.*',  # European Conference on Computer Vision
            r'.*aaai.*',  # Association for the Advancement of Artificial Intelligence
            r'.*ijcai.*', # International Joint Conference on Artificial Intelligence
            r'.*acm.*',   # Association for Computing Machinery
            r'.*ieee.*',  # Institute of Electrical and Electronics Engineers

            # Journals
            r'.*journal.*',  # General journal pattern
            r'.*transactions.*',
            r'.*letters.*',
            r'.*international.*journal.*',
            r'.*nature.*',
            r'.*science.*',
            r'.*springer.*',
            r'.*elsevier.*',
            r'.*wiley.*',
            r'.*mit.*press.*',
            r'.*frontiers.*',
            r'.*plos.*',

            # Specific robotics journals
            r'.*robotics.*research.*',
            r'.*autonomous.*robots.*',
            r'.*field.*robotics.*',
            r'.*robotics.*automation.*',
            r'.*humanoid.*robotics.*',
        ]

        # Known non-peer-reviewed patterns
        self.non_peer_reviewed_patterns = [
            r'.*arxiv\.org.*',      # Preprint server
            r'.*github\.com.*',     # Code repositories
            r'.*youtube\.com.*',    # Video content
            r'.*wikipedia\.org.*',  # Wiki content
            r'.*medium\.com.*',     # Blog platform
            r'.*towardsdatascience\.com.*',  # Blog platform
            r'.*personal\.website.*',  # Personal websites
            r'.*company\.com.*',    # Company websites
        ]

    def run_test(self) -> bool:
        """Run the peer-reviewed source ratio test."""
        print("🔍 Running Peer-Reviewed Source Ratio Test...\n")

        # Check if references directory exists
        if not self.references_dir.exists():
            error_msg = f"References directory does not exist: {self.references_dir}"
            self.results["failed"].append(error_msg)
            print(f"❌ {error_msg}")
            return False

        # Collect all reference files
        reference_files = list(self.references_dir.glob("*.md")) + list(self.references_dir.glob("*.bib"))

        if not reference_files:
            warn_msg = f"No reference files found in {self.references_dir}"
            self.results["warnings"].append(warn_msg)
            print(f"⚠️  {warn_msg}")
            return True  # Not a failure, just no references to check

        print(f"📄 Found {len(reference_files)} reference files to analyze\n")

        # Process each reference file
        for ref_file in reference_files:
            print(f"  Analyzing {ref_file.name}...")
            self.analyze_reference_file(ref_file)
            print(f"    Processed {ref_file.name}\n")

        # Calculate and evaluate the ratio
        self.calculate_ratio()
        is_passing = self.evaluate_ratio()

        # Print results
        self.print_results()

        return is_passing

    def analyze_reference_file(self, file_path: Path):
        """Analyze a single reference file for peer-reviewed sources."""
        if file_path.suffix.lower() == '.bib':
            self.analyze_bibtex_file(file_path)
        elif file_path.suffix.lower() == '.md':
            self.analyze_markdown_file(file_path)

    def analyze_bibtex_file(self, file_path: Path):
        """Analyze a BibTeX file for peer-reviewed sources."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                bib_database = bibtexparser.load(f)

            for entry in bib_database.entries:
                self.results["stats"]["total_sources"] += 1

                # Check if this is a peer-reviewed source
                is_peer_reviewed = self.is_peer_reviewed_bibtex(entry)

                if is_peer_reviewed:
                    self.results["stats"]["peer_reviewed"] += 1
                    self.results["passed"].append(f"Peer-reviewed: {entry.get('title', 'Unknown title')} ({entry.get('author', 'Unknown author')})")
                    print(f"    ✅ {entry.get('title', 'Unknown title')[:50]}...")
                else:
                    self.results["stats"]["non_peer_reviewed"] += 1
                    self.results["warnings"].append(f"Non-peer-reviewed: {entry.get('title', 'Unknown title')} ({entry.get('author', 'Unknown author')})")
                    print(f"    ⚠️  {entry.get('title', 'Unknown title')[:50]}...")

        except Exception as e:
            error_msg = f"Error analyzing BibTeX file {file_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"    ❌ Error: {str(e)}")

    def analyze_markdown_file(self, file_path: Path):
        """Analyze a Markdown file for peer-reviewed sources."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Find all markdown links which might be references
            # This is a simple approach - could be enhanced to parse proper citation formats
            links = re.findall(r'\[([^\]]+)\]\(([^)]+)\)', content)

            for link_text, url in links:
                self.results["stats"]["total_sources"] += 1

                # Check if this is a peer-reviewed source based on URL
                is_peer_reviewed = self.is_peer_reviewed_url(url)

                if is_peer_reviewed:
                    self.results["stats"]["peer_reviewed"] += 1
                    self.results["passed"].append(f"Peer-reviewed: {link_text} -> {url}")
                    print(f"    ✅ {link_text[:50]}...")
                else:
                    self.results["stats"]["non_peer_reviewed"] += 1
                    self.results["warnings"].append(f"Non-peer-reviewed: {link_text} -> {url}")
                    print(f"    ⚠️  {link_text[:50]}...")

            # Also look for APA-style citations in the text
            apa_citations = self.find_apa_citations(content)
            for citation in apa_citations:
                self.results["stats"]["total_sources"] += 1

                # For APA citations, we'll assume they are from academic sources
                # but we can't be certain without checking the actual publication
                # For now, we'll categorize them as potentially peer-reviewed
                self.results["stats"]["peer_reviewed"] += 1
                self.results["passed"].append(f"Potential peer-reviewed: {citation}")
                print(f"    📚 APA citation: {citation[:50]}...")

        except Exception as e:
            error_msg = f"Error analyzing Markdown file {file_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"    ❌ Error: {str(e)}")

    def find_apa_citations(self, text: str) -> List[str]:
        """Find potential APA-style citations in text."""
        # Pattern for APA citations: (Author, Year) or Author (Year)
        patterns = [
            r'\([A-Z][a-z]+,\s*\d{4}\)',  # (Smith, 2023)
            r'[A-Z][a-z]+\s*\([A-Z][a-z]*\s*\d{4}\)',  # Smith (2023)
            r'\([A-Z][a-z]+,\s*[A-Z][a-z]+,\s*\d{4}\)',  # (Smith, Johnson, 2023)
        ]

        citations = []
        for pattern in patterns:
            matches = re.findall(pattern, text)
            citations.extend(matches)

        return citations

    def is_peer_reviewed_bibtex(self, entry: Dict) -> bool:
        """Check if a BibTeX entry is likely peer-reviewed."""
        # Check entry type - certain types are more likely to be peer-reviewed
        entry_type = entry.get('ENTRYTYPE', '').lower()
        if entry_type in ['article', 'inproceedings', 'book', 'incollection']:
            # These types are typically peer-reviewed
            return True
        elif entry_type in ['unpublished', 'techreport', 'mastersthesis', 'phdthesis']:
            # These might not be peer-reviewed
            return False

        # Check for conference/journal name patterns
        booktitle = entry.get('booktitle', '').lower()
        journal = entry.get('journal', '').lower()

        # Check if it matches peer-reviewed patterns
        full_text = f"{booktitle} {journal}".lower()
        for pattern in self.peer_reviewed_patterns:
            if re.search(pattern, full_text):
                return True

        # Check if it matches non-peer-reviewed patterns
        for pattern in self.non_peer_reviewed_patterns:
            if re.search(pattern, full_text):
                return False

        # Default assumption for articles and inproceedings
        if entry_type in ['article', 'inproceedings']:
            return True

        return False

    def is_peer_reviewed_url(self, url: str) -> bool:
        """Check if a URL is likely from a peer-reviewed source."""
        # Check against non-peer-reviewed patterns first
        for pattern in self.non_peer_reviewed_patterns:
            if re.search(pattern, url.lower()):
                return False

        # Check against peer-reviewed patterns
        for pattern in self.peer_reviewed_patterns:
            if re.search(pattern, url.lower()):
                return True

        # Check for common academic domain patterns
        parsed = urlparse(url)
        domain = parsed.netloc.lower()

        academic_indicators = [
            '.edu', '.ac.', '.edu.',  # Educational institutions
            'springer', 'elsevier', 'wiley',  # Academic publishers
            'ieee', 'acm',  # Professional organizations
            'nature.com', 'science.org',  # Prestigious journals
        ]

        for indicator in academic_indicators:
            if indicator in domain:
                return True

        # Default: assume not peer-reviewed if no clear indicator
        return False

    def calculate_ratio(self):
        """Calculate the peer-reviewed source ratio."""
        if self.results["stats"]["total_sources"] > 0:
            self.results["stats"]["ratio"] = (
                self.results["stats"]["peer_reviewed"] /
                self.results["stats"]["total_sources"]
            )
        else:
            self.results["stats"]["ratio"] = 0.0

    def evaluate_ratio(self) -> bool:
        """Evaluate if the ratio meets the required threshold."""
        actual_ratio = self.results["stats"]["ratio"]
        required_ratio = self.required_ratio

        if actual_ratio >= required_ratio:
            success_msg = f"Peer-reviewed ratio meets requirement: {actual_ratio:.1%} >= {required_ratio:.1%}"
            self.results["passed"].append(success_msg)
            return True
        else:
            failure_msg = f"Peer-reviewed ratio below requirement: {actual_ratio:.1%} < {required_ratio:.1%}"
            self.results["failed"].append(failure_msg)
            return False

    def print_results(self):
        """Print the test results."""
        print("="*60)
        print("PEER-REVIEWED SOURCE RATIO TEST RESULTS")
        print("="*60)

        stats = self.results["stats"]
        print(f"Total sources analyzed: {stats['total_sources']}")
        print(f"Peer-reviewed sources: {stats['peer_reviewed']}")
        print(f"Non peer-reviewed sources: {stats['non_peer_reviewed']}")
        print(f"Peer-reviewed ratio: {stats['ratio']:.1%}")
        print(f"Required ratio: {self.required_ratio:.1%}")

        print(f"\nPassed checks: {len(self.results['passed'])}")
        print(f"Failed checks: {len(self.results['failed'])}")
        print(f"Warnings: {len(self.results['warnings'])}")

        if self.results["failed"]:
            print(f"\n❌ FAILED CHECKS:")
            for failure in self.results["failed"]:
                print(f"  • {failure}")

        if self.results["warnings"]:
            print(f"\n⚠️  WARNINGS:")
            for warning in self.results["warnings"]:
                print(f"  • {warning}")

        if self.results["passed"]:
            print(f"\n✅ PASSED CHECKS (showing first 5):")
            for passed in self.results["passed"][:5]:
                print(f"  • {passed}")
            if len(self.results["passed"]) > 5:
                print(f"  ... and {len(self.results["passed"]) - 5} more")

        success_rate = (len(self.results["passed"]) / max(len(self.results["passed"]) + len(self.results["failed"]), 1)) * 100
        print(f"\nSuccess Rate: {success_rate:.1f}%")

        if not self.results["failed"]:
            print(f"\n🎉 Peer-reviewed source ratio requirement met!")
            sys.exit(0)
        else:
            print(f"\n❌ Peer-reviewed source ratio requirement not met.")
            sys.exit(1)

def main():
    """Main function to run the peer-reviewed source checker."""
    print("Humanoid Robotics Book - Peer-Reviewed Source Checker")
    print("="*55)

    # Create tests directory if it doesn't exist
    tests_dir = Path("tests/quality-assurance")
    tests_dir.mkdir(parents=True, exist_ok=True)

    # Initialize and run checker
    checker = PeerReviewedSourceChecker()
    is_passing = checker.run_test()

    return is_passing

if __name__ == "__main__":
    main()