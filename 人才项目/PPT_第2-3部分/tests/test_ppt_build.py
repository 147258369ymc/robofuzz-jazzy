import hashlib
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
TOOLS = ROOT / "tools"
SOURCE = REPO / "人才项目" / "拔尖人才项目阶段进展汇报.pptx"

sys.path.insert(0, str(TOOLS))

import uno_ppt


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class UnoPptTests(unittest.TestCase):
    def test_enter_removes_profile_if_socket_creation_fails(self):
        with tempfile.TemporaryDirectory() as temporary_directory:
            profile = Path(temporary_directory) / "profile"
            profile.mkdir()
            session = uno_ppt.ImpressSession()
            with (
                patch.object(uno_ppt.tempfile, "mkdtemp", return_value=str(profile)),
                patch.object(
                    uno_ppt.socket, "socket", side_effect=PermissionError("blocked")
                ),
                self.assertRaises(PermissionError),
            ):
                session.__enter__()
            self.assertFalse(profile.exists())

    def test_editing_copy_does_not_change_source(self):
        source_hash = sha256(SOURCE)
        with tempfile.TemporaryDirectory() as temporary_directory:
            copy_path = Path(temporary_directory) / "copy.pptx"
            shutil.copy2(SOURCE, copy_path)

            with uno_ppt.ImpressSession() as session:
                document = session.open(copy_path)
                self.assertEqual(document.getDrawPages().getCount(), 8)
                document.getDrawPages().remove(document.getDrawPages().getByIndex(0))
                session.save_and_close(document)

            with uno_ppt.ImpressSession() as session:
                document = session.open(copy_path)
                self.assertEqual(document.getDrawPages().getCount(), 7)
                session.close(document)

        self.assertEqual(sha256(SOURCE), source_hash)


if __name__ == "__main__":
    unittest.main()
