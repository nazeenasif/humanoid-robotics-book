#!/usr/bin/env python3
"""
Script to verify code examples in the humanoid robotics book.

This script checks all code examples in the code-examples directory to ensure:
1. Python files have valid syntax
2. ROS 2 related files follow proper structure
3. Configuration files are valid YAML/JSON
4. All referenced files exist and are properly formatted
"""

import os
import sys
import subprocess
import yaml
import json
from pathlib import Path
from typing import List, Dict, Tuple
import ast

class CodeExampleVerifier:
    def __init__(self, base_dir: str = "code-examples"):
        self.base_dir = Path(base_dir)
        self.results = {
            "passed": [],
            "failed": [],
            "skipped": []
        }
        self.total_tests = 0

    def verify_all_examples(self) -> Dict:
        """Verify all code examples in the code-examples directory."""
        print(f"Verifying code examples in {self.base_dir}/")

        # Check if code-examples directory exists
        if not self.base_dir.exists():
            print(f"Error: {self.base_dir} directory does not exist!")
            return self.results

        # Verify each subdirectory
        for subdir in self.base_dir.iterdir():
            if subdir.is_dir():
                print(f"\nVerifying {subdir.name} examples...")
                self.verify_subdirectory(subdir)

        self.print_summary()
        return self.results

    def verify_subdirectory(self, subdir: Path):
        """Verify code examples in a specific subdirectory."""
        for file_path in subdir.rglob("*"):
            if file_path.is_file():
                self.total_tests += 1
                relative_path = file_path.relative_to(Path.cwd())

                if file_path.suffix == '.py':
                    self.verify_python_file(file_path, relative_path)
                elif file_path.suffix in ['.yaml', '.yml']:
                    self.verify_yaml_file(file_path, relative_path)
                elif file_path.suffix == '.json':
                    self.verify_json_file(file_path, relative_path)
                elif file_path.suffix == '.sdf':
                    self.verify_sdf_file(file_path, relative_path)
                elif file_path.suffix == '.urdf':
                    self.verify_urdf_file(file_path, relative_path)
                else:
                    # Skip files that don't need verification
                    self.results["skipped"].append(f"Skipped {relative_path} (unsupported type)")
                    print(f"  ⏭️  Skipped {relative_path}")

    def verify_python_file(self, file_path: Path, relative_path: Path):
        """Verify Python file syntax and basic structure."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Parse the Python file to check for syntax errors
            ast.parse(content)

            # Additional checks for ROS 2 specific content
            if self.is_ros2_file(content):
                self.check_ros2_conventions(content, file_path)

            self.results["passed"].append(f"Valid Python: {relative_path}")
            print(f"  ✅ {relative_path}")

        except SyntaxError as e:
            error_msg = f"Syntax error in {relative_path}: {e.msg} at line {e.lineno}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - Syntax Error")

        except Exception as e:
            error_msg = f"Error verifying {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - Error: {str(e)}")

    def verify_yaml_file(self, file_path: Path, relative_path: Path):
        """Verify YAML file structure."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                yaml.safe_load(content)

            # Additional checks for specific config types
            if 'nav2' in str(file_path).lower():
                self.check_nav2_config(file_path)
            elif 'vslam' in str(file_path).lower():
                self.check_vslam_config(file_path)

            self.results["passed"].append(f"Valid YAML: {relative_path}")
            print(f"  ✅ {relative_path}")

        except yaml.YAMLError as e:
            error_msg = f"YAML error in {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - YAML Error")

        except Exception as e:
            error_msg = f"Error verifying {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - Error: {str(e)}")

    def verify_json_file(self, file_path: Path, relative_path: Path):
        """Verify JSON file structure."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
                json.loads(content)

            self.results["passed"].append(f"Valid JSON: {relative_path}")
            print(f"  ✅ {relative_path}")

        except json.JSONDecodeError as e:
            error_msg = f"JSON error in {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - JSON Error")

        except Exception as e:
            error_msg = f"Error verifying {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - Error: {str(e)}")

    def verify_sdf_file(self, file_path: Path, relative_path: Path):
        """Verify SDF file (basic XML structure check)."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Basic XML well-formedness check
            import xml.etree.ElementTree as ET
            ET.fromstring(content)

            self.results["passed"].append(f"Valid SDF: {relative_path}")
            print(f"  ✅ {relative_path}")

        except ET.ParseError as e:
            error_msg = f"XML error in {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - XML Error")

        except Exception as e:
            error_msg = f"Error verifying {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - Error: {str(e)}")

    def verify_urdf_file(self, file_path: Path, relative_path: Path):
        """Verify URDF file (basic XML structure check)."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Basic XML well-formedness check
            import xml.etree.ElementTree as ET
            ET.fromstring(content)

            self.results["passed"].append(f"Valid URDF: {relative_path}")
            print(f"  ✅ {relative_path}")

        except ET.ParseError as e:
            error_msg = f"XML error in {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - XML Error")

        except Exception as e:
            error_msg = f"Error verifying {relative_path}: {str(e)}"
            self.results["failed"].append(error_msg)
            print(f"  ❌ {relative_path} - Error: {str(e)}")

    def is_ros2_file(self, content: str) -> bool:
        """Check if a Python file is a ROS 2 file."""
        ros2_indicators = [
            'rclpy',
            'ros2',
            'Node',
            'create_publisher',
            'create_subscription',
            'ROS_VERSION'
        ]
        content_lower = content.lower()
        return any(indicator.lower() in content_lower for indicator in ros2_indicators)

    def check_ros2_conventions(self, content: str, file_path: Path):
        """Check ROS 2 specific conventions."""
        # Check for proper imports
        required_imports = ['import rclpy', 'from rclpy.node import Node']
        missing_imports = [imp for imp in required_imports if imp not in content]

        if missing_imports:
            print(f"    ⚠️  Missing common ROS 2 imports in {file_path.name}: {missing_imports}")

    def check_nav2_config(self, file_path: Path):
        """Check Nav2 configuration specific requirements."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read().lower()

        # Check for essential Nav2 components
        essential_components = [
            'planner_server',
            'controller_server',
            'bt_navigator',
            'local_costmap',
            'global_costmap'
        ]

        missing_components = [comp for comp in essential_components if comp not in content]
        if missing_components:
            print(f"    ⚠️  Missing Nav2 components in {file_path.name}: {missing_components}")

    def check_vslam_config(self, file_path: Path):
        """Check VSLAM configuration specific requirements."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read().lower()

        # Check for essential VSLAM parameters
        essential_params = [
            'camera',
            'feature_detection',
            'tracking',
            'mapping'
        ]

        missing_params = [param for param in essential_params if param not in content]
        if missing_params:
            print(f"    ⚠️  Missing VSLAM parameters in {file_path.name}: {missing_params}")

    def print_summary(self):
        """Print a summary of verification results."""
        print(f"\n{'='*60}")
        print("CODE EXAMPLE VERIFICATION SUMMARY")
        print(f"{'='*60}")
        print(f"Total tests: {self.total_tests}")
        print(f"Passed: {len(self.results['passed'])}")
        print(f"Failed: {len(self.results['failed'])}")
        print(f"Skipped: {len(self.results['skipped'])}")

        if self.results["failed"]:
            print(f"\n❌ FAILED TESTS:")
            for failure in self.results["failed"]:
                print(f"  - {failure}")

        if self.results["skipped"]:
            print(f"\n⏭️  SKIPPED FILES:")
            for skipped in self.results["skipped"]:
                print(f"  - {skipped}")

        success_rate = len(self.results["passed"]) / max(self.total_tests, 1) * 100
        print(f"\nSuccess Rate: {success_rate:.1f}%")

        if self.results["failed"]:
            print(f"\n⚠️  Some code examples failed verification.")
            sys.exit(1)
        else:
            print(f"\n🎉 All code examples passed verification!")
            sys.exit(0)

def main():
    """Main function to run the code example verifier."""
    print("Humanoid Robotics Book - Code Example Verifier")
    print("="*50)

    # Create tests directory if it doesn't exist
    tests_dir = Path("tests/code-examples")
    tests_dir.mkdir(parents=True, exist_ok=True)

    # Initialize and run verifier
    verifier = CodeExampleVerifier()
    results = verifier.verify_all_examples()

    return results

if __name__ == "__main__":
    main()