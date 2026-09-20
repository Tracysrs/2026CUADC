"""dataset_audit 单元测试（纯 stdlib，构造临时好/坏样本）：

    cd 01_视觉感知/数据与训练
    python -m unittest test_dataset_audit -v

每类审计项都有正反用例：审计器漏报 = 防线不存在。
"""

import tempfile
import unittest
from pathlib import Path

from dataset_audit import audit_dataset, audit_manifest

JPEG_MAGIC = b'\xff\xd8\xff\xe0' + b'\x00' * 32   # 伪 JPEG（审计只看 md5，不解码）


class AuditTestBase(unittest.TestCase):

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        self.images = self.root / 'images'
        self.labels = self.root / 'labels'
        self.images.mkdir()
        self.labels.mkdir()

    def tearDown(self):
        self._tmp.cleanup()

    def add_image(self, stem: str, content: bytes = JPEG_MAGIC):
        (self.images / f'{stem}.jpg').write_bytes(content)

    def add_label(self, stem: str, text: str):
        (self.labels / f'{stem}.txt').write_text(text, encoding='utf-8')

    def audit(self, classes=3, task='detect'):
        return audit_dataset(self.images, self.labels, classes, task)


class TestHappyPath(AuditTestBase):

    def test_clean_detect_dataset_passes(self):
        self.add_image('a')
        self.add_image('b', content=JPEG_MAGIC + b'\x01')
        self.add_label('a', '0 0.5 0.5 0.2 0.2\n')
        self.add_label('b', '2 0.25 0.25 0.1 0.1\n1 0.75 0.75 0.3 0.3\n')
        errors, warnings, stats = self.audit()
        self.assertEqual(errors, [])
        self.assertEqual(warnings, [])
        self.assertEqual(stats['paired'], 2)
        self.assertEqual(stats['boxes'], 3)

    def test_empty_label_is_negative_sample_warning(self):
        """空标签 = 有意负样本：合法但须人工确认，不算 error。"""
        self.add_image('neg')
        self.add_label('neg', '')
        errors, warnings, _ = self.audit()
        self.assertEqual(errors, [])
        self.assertEqual(len(warnings), 1)
        self.assertIn('负样本', warnings[0])

    def test_clean_segment_dataset_passes(self):
        self.add_image('s')
        # cls + 4 点 (8 值)
        self.add_label('s', '0 0.1 0.1 0.2 0.1 0.2 0.2 0.1 0.2\n')
        errors, _, stats = self.audit(task='segment')
        self.assertEqual(errors, [])
        self.assertEqual(stats['boxes'], 1)


class TestLabelErrors(AuditTestBase):

    def test_missing_label_and_missing_image(self):
        self.add_image('no_label')
        self.add_label('no_image', '0 0.5 0.5 0.2 0.2\n')
        errors, _, _ = self.audit()
        self.assertTrue(any('图片缺标签: no_label' in e for e in errors))
        self.assertTrue(any('标签缺图片: no_image' in e for e in errors))

    def test_class_out_of_range(self):
        self.add_image('a')
        self.add_label('a', '5 0.5 0.5 0.2 0.2\n')     # classes=3 → 越界
        errors, _, _ = self.audit(classes=3)
        self.assertTrue(any('类别越界' in e for e in errors))

    def test_nan_and_nonnumeric(self):
        self.add_image('a')
        self.add_label('a', '0 nan 0.5 0.2 0.2\nfoo bar baz qux\n')
        errors, _, _ = self.audit()
        self.assertTrue(any('NaN/Inf' in e for e in errors))
        self.assertTrue(any('非数值字段' in e for e in errors))

    def test_coordinate_out_of_range_and_zero_box(self):
        self.add_image('a')
        self.add_label('a', '0 1.5 0.5 0.2 0.2\n0 0.5 0.5 0.0 0.2\n')
        errors, _, _ = self.audit()
        self.assertTrue(any('坐标越界' in e for e in errors))
        self.assertTrue(any('零面积' in e for e in errors))

    def test_segment_polygon_too_few_points(self):
        self.add_image('s')
        self.add_label('s', '0 0.1 0.1 0.2 0.2\n')     # 2 点 < 3
        errors, _, _ = self.audit(task='segment')
        self.assertTrue(any('点数 2 < 3' in e for e in errors))

    def test_segment_odd_coordinate_count(self):
        self.add_image('s')
        self.add_label('s', '0 0.1 0.1 0.2 0.1 0.3\n')  # 5 个坐标值
        errors, _, _ = self.audit(task='segment')
        self.assertTrue(any('奇数' in e for e in errors))


class TestDuplicatesAndManifest(AuditTestBase):

    def test_exact_duplicate_image_detected(self):
        self.add_image('a')
        self.add_image('b')                            # 同字节 → md5 相同
        self.add_image('c', content=JPEG_MAGIC + b'\x02')
        self.add_label('a', '0 0.5 0.5 0.2 0.2\n')
        self.add_label('b', '0 0.5 0.5 0.2 0.2\n')
        self.add_label('c', '0 0.5 0.5 0.2 0.2\n')
        errors, _, stats = self.audit()
        self.assertTrue(any('精确重复图片: b.jpg 与 a.jpg' in e for e in errors))
        self.assertEqual(stats['duplicates'], 1)

    def test_manifest_session_leak_detected(self):
        with tempfile.TemporaryDirectory() as td:
            manifest = Path(td) / 'manifest.csv'
            manifest.write_text(
                'image,label,split,session\n'
                'a.jpg,a.txt,train,s1\n'
                'b.jpg,b.txt,train,s1\n'
                'c.jpg,c.txt,val,s1\n'                 # s1 泄漏到 val
                'd.jpg,d.txt,val,s2\n',
                encoding='utf-8')
            errors, sessions = audit_manifest(manifest)
            self.assertEqual(len(sessions), 2)
            self.assertTrue(any('session 泄漏' in e and 's1' in e for e in errors))

    def test_manifest_missing_column(self):
        with tempfile.TemporaryDirectory() as td:
            manifest = Path(td) / 'bad.csv'
            manifest.write_text('image,label,split\n', encoding='utf-8')
            errors, _ = audit_manifest(manifest)
            self.assertTrue(any('缺列' in e for e in errors))


if __name__ == '__main__':
    unittest.main()
