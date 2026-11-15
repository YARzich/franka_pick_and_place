"""
Configure source-tree imports for the consolidated perception tests.

Responsibility: Expose the package source root without changing installed behavior.
"""

from pathlib import Path
import sys


TEST_ROOT = Path(__file__).resolve().parent
PACKAGE_SOURCE_ROOT = TEST_ROOT.parent
for import_root in (PACKAGE_SOURCE_ROOT, TEST_ROOT):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))
