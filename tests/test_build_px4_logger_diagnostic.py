"""独立 PX4 logger 诊断构建的链接参数边界。"""

import importlib.util
from pathlib import Path, PurePosixPath
import unittest

SPEC = importlib.util.spec_from_file_location('logger_builder', Path(__file__).resolve().parents[1] /
                                             'scripts/build_px4_logger_diagnostic.py')
builder = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(builder)


class DiagnosticLinkTests(unittest.TestCase):
    def test_compound_ninja_command_preserves_space_arguments(self):
        raw = (': && /usr/bin/c++ "source with space.o" -o bin/px4 '
               'src/modules/logger/libmodules__logger.a "other library.a" '
               '&& cd bin && /usr/bin/cmake -E create_symlink px4 px4-logger')
        result = builder.link_command(raw, PurePosixPath('/tmp/trace build/logger.a'), PurePosixPath('/tmp/trace build/px4'))
        self.assertEqual(result, ['/usr/bin/c++', 'source with space.o', '-o', '/tmp/trace build/px4',
                                  '/tmp/trace build/logger.a', 'other library.a'])

    def test_no_recognizable_link_is_rejected(self):
        with self.assertRaisesRegex(ValueError, '单个可识别'):
            builder.link_command('cmake -E echo none', Path('a'), Path('b'))

    def test_multiple_link_steps_are_rejected(self):
        raw = '/usr/bin/c++ -o a x && /usr/bin/c++ -o b y'
        with self.assertRaisesRegex(ValueError, '单个可识别'):
            builder.link_command(raw, Path('a'), Path('b'))

    def test_missing_or_repeated_logger_archive_is_rejected(self):
        for count in (0, 2):
            raw = '/usr/bin/c++ -o bin/px4 ' + 'src/modules/logger/libmodules__logger.a ' * count
            with self.subTest(count=count), self.assertRaisesRegex(ValueError, '单个 logger'):
                builder.link_command(raw, Path('a'), Path('b'))

    def test_extra_shell_operation_in_link_step_is_rejected(self):
        raw = '/usr/bin/c++ -o bin/px4 src/modules/logger/libmodules__logger.a | other'
        with self.assertRaisesRegex(ValueError, 'shell'):
            builder.link_command(raw, Path('a'), Path('b'))


if __name__ == '__main__':
    unittest.main()
