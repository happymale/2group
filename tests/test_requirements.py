import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('requirements', Path(__file__).parents[1] / 'tools/requirements.py')
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


def req(uid, status='Draft', parent=None, category=None):
    result = f'\n[REQUIREMENT]\nUID: {uid}\nTITLE: Example\nSTATEMENT: The user shall repeat a run.\nSTATUS: {status}\n'
    if category: result += f'CATEGORY: {category}\n'
    if parent: result += f'RELATIONS:\n- TYPE: Parent\n  VALUE: {parent}\n'
    return result


class FakeGitHub:
    def __init__(self):
        self.issue = {'number': 1, 'state': 'closed', 'body': r.with_block('User text', ['SW-001'])}
        self.pr = {'number': 2, 'body': 'Human PR description\nResolves #1'}
        self.writes = []
    def pages(self, endpoint):
        return [self.issue] if '/issues?' in endpoint else [self.pr]
    def linked(self, repo, number): return [self.issue]
    def call(self, endpoint, payload=None):
        if payload:
            self.writes.append(payload)
            self.pr.update(payload)
        return self.pr.copy()


class MultiGitHub:
    def __init__(self):
        self.issues, self.prs, self.links, self.writes = {}, {}, {}, []
    def add_issue(self, number, state, body):
        self.issues[number] = {'number': number, 'state': state, 'body': body}
    def add_pr(self, number, body, links):
        self.prs[number] = {'number': number, 'body': body}
        self.links[number] = links
    def pages(self, endpoint):
        return list((self.issues if '/issues?' in endpoint else self.prs).values())
    def linked(self, repo, number):
        return [self.issues[n] for n in self.links[number]]
    def call(self, endpoint, payload=None):
        pr = self.prs[int(endpoint.rsplit('/', 1)[1])]
        if payload:
            self.writes.append(payload)
            pr.update(payload)
        return pr.copy()


class RequirementsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
    def tearDown(self): self.temp.cleanup()
    def initialize(self, levels=('stakeholder', 'software')):
        r.initialize(self.root, list(levels))
    def add(self, level, text):
        path = self.root / f'docs/requirements/{level}.sdoc'
        path.write_text(path.read_text() + text)
    def test_init_preserves_existing_content(self):
        self.initialize()
        self.add('stakeholder', req('STK-001'))
        before = {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()}
        self.initialize()
        self.assertEqual(before, {p: p.read_bytes() for p in self.root.rglob('*') if p.is_file()})
    def test_init_adds_only_missing_level_when_selection_grows(self):
        self.initialize(['software'])
        self.add('software', req('SW-001', category='Functional'))
        before = (self.root / 'docs/requirements/software.sdoc').read_text()
        self.initialize(['stakeholder', 'software'])
        self.assertEqual(before, (self.root / 'docs/requirements/software.sdoc').read_text())
        self.assertTrue((self.root / 'docs/requirements/stakeholder.sdoc').exists())
    def test_configuration_requires_levels_only_when_enabled(self):
        def config(body):
            (self.root / 'heven.toml').write_text(body)
            return r.configuration(self.root)
        self.assertFalse(config('[requirements]\nenabled=false\nlevels=[]\n')['enabled'])
        for bad in ('levels=[]', 'levels=["software","unknown"]', 'levels=["software","software"]'):
            with self.assertRaises(ValueError): config(f'[requirements]\nenabled=true\n{bad}\n')
        self.assertEqual(['system', 'software', 'hardware'],
            config('[requirements]\nenabled=true\nlevels=["hardware","software","system"]\n')['levels'])
    def test_hardware_and_software_are_siblings_under_system(self):
        levels = ['system', 'software', 'hardware']
        self.initialize(levels)
        self.add('system', req('SYS-001', 'Approved'))
        self.add('software', req('SW-001', 'Approved', 'SYS-001', 'Functional'))
        self.add('hardware', req('HW-001', 'Approved', 'SYS-001'))
        self.assertEqual([], r.validate(self.root, levels)[1])
    def test_approved_hardware_needs_parent_from_nearest_selected_level(self):
        levels = ['stakeholder', 'software', 'hardware']
        self.initialize(levels)
        self.add('hardware', req('HW-001', 'Approved'))
        self.assertIn('needs a parent at stakeholder', str(r.validate(self.root, levels)[1]))
    def test_retired_prefixes_are_not_treated_as_levels(self):
        self.initialize(['software'])
        self.add('software', req('STR-001', category='Functional'))
        self.assertEqual('software', r.validate(self.root, ['software'])[0]['STR-001']['level'])
    def test_draft_partial_then_approved_relations(self):
        self.initialize()
        self.add('stakeholder', req('STK-001'))
        self.add('software', req('SW-001', category='Functional'))
        self.assertFalse(r.validate(self.root, ['stakeholder', 'software'])[1])
        path = self.root / 'docs/requirements/software.sdoc'
        path.write_text(path.read_text().replace('STATUS: Draft', 'STATUS: Approved'))
        self.assertTrue(r.validate(self.root, ['stakeholder', 'software'])[1])
        path.write_text(path.read_text() + 'RELATIONS:\n- TYPE: Parent\n  VALUE: STK-001\n')
        self.assertFalse(r.validate(self.root, ['stakeholder', 'software'])[1])
    def test_duplicate_unknown_parent_category_and_retired(self):
        self.initialize()
        self.add('software', req('SW-001', 'Retired', 'MISSING-1', 'bad'))
        self.add('software', req('SW-001', category='Functional'))
        errors = str(r.validate(self.root, ['stakeholder', 'software'])[1])
        for word in ('duplicate', 'CATEGORY', 'retirement', 'does not exist'):
            self.assertIn(word, errors)
    def test_each_level_template_and_software_only_approval(self):
        for level in r.LEVELS:
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                r.initialize(root, [level])
                self.assertEqual({level}, set(p.stem for p in (root / 'docs/requirements').glob('*.sdoc')))
        self.initialize(['software'])
        self.add('software', req('SW-001', 'Approved', category='Functional'))
        self.assertFalse(r.validate(self.root, ['software'])[1])
    def test_existing_uid_is_preserved_by_document_level(self):
        self.initialize(['software'])
        self.add('software', req('REQ-LEGACY-001', category='Functional'))
        index, errors = r.validate(self.root, ['software'])
        self.assertFalse(errors)
        self.assertIn('REQ-LEGACY-001', index)

    def test_cyclic_relations(self):
        self.initialize()
        self.add('stakeholder', req('STK-001', parent='SW-001'))
        self.add('software', req('SW-001', parent='STK-001', category='Functional'))
        self.assertIn('cyclic', str(r.validate(self.root, ['stakeholder', 'software'])[1]))
    def test_blocks_preserve_text_empty_and_ambiguous(self):
        body = 'Before\n' + r.with_block('', ['SW-002', 'SW-001']) + '\nAfter'
        new = r.with_block(body, [])
        self.assertTrue(new.startswith('Before\n'))
        self.assertTrue(new.endswith('\nAfter'))
        self.assertEqual([], r.block_ids(new))
        self.assertEqual(new, r.with_block(new, []))
        with self.assertRaises(ValueError): r.with_block(body + r.START, [])
    def test_sync_closed_issue_union_idempotency_and_unknown(self):
        api = FakeGitHub()
        r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertEqual(1, len(api.writes))
        self.assertTrue(api.pr['body'].startswith('Human PR description\nResolves #1'))
        self.assertEqual(['SW-001'], r.block_ids(api.pr['body']))
        r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertEqual(1, len(api.writes))
        api.issue['state'] = 'open'
        api.issue['body'] = r.with_block('', ['SW-UNKNOWN'])
        result = r.synchronize(self.root, {}, 'a/b', api, True)
        self.assertIn('SW-UNKNOWN', str(result['errors']))
        self.assertEqual(1, len(api.writes))
    def test_sync_bad_open_issue_does_not_block_other_prs(self):
        api = MultiGitHub()
        api.add_issue(1, 'open', r.START + '\nnot-a-uid\n' + r.END)
        api.add_issue(2, 'open', r.with_block('', ['SW-001']))
        api.add_pr(10, 'First\nResolves #1', [1])
        api.add_pr(20, 'Second\nResolves #2', [2])
        result = r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertEqual(['SW-001'], r.block_ids(api.prs[20]['body']))
        self.assertEqual('First\nResolves #1', api.prs[10]['body'])
        self.assertEqual(2, len(result['errors']))
    def test_sync_closed_issue_with_removed_uid_only_warns(self):
        api = MultiGitHub()
        api.add_issue(1, 'closed', r.with_block('', ['SW-OLD']))
        api.add_issue(2, 'open', r.with_block('', ['SW-001']))
        api.add_pr(20, 'Second\nResolves #2', [2])
        result = r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertEqual([], result['errors'])
        self.assertIn('SW-OLD', str(result['warnings']))
        self.assertEqual(['SW-001'], r.block_ids(api.prs[20]['body']))
    def test_sync_pr_without_uids_keeps_body_unchanged(self):
        api = MultiGitHub()
        api.add_pr(30, 'No linked issue', [])
        r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertEqual('No linked issue', api.prs[30]['body'])
        self.assertEqual([], api.writes)
    def test_sync_empties_existing_block_when_links_are_removed(self):
        api = MultiGitHub()
        api.add_pr(31, r.with_block('Body', ['SW-001']), [])
        r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertEqual([], r.block_ids(api.prs[31]['body']))
    def test_missing_selected_document_fails_check_without_creating_it(self):
        import subprocess, sys
        (self.root / 'heven.toml').write_text('[requirements]\nenabled=true\nlevels=["software"]\n')
        result = subprocess.run([sys.executable, str(r.ROOT/'tools/requirements.py'), 'check', '--root', str(self.root)],
            capture_output=True, text=True)
        self.assertEqual(1, result.returncode)
        self.assertIn('requirements-init', result.stderr)
        self.assertFalse((self.root / 'docs/requirements/software.sdoc').exists())
    def test_sync_preserves_edit_during_link_lookup(self):
        api = FakeGitHub()
        original = api.linked
        calls = []
        def linked(repo, number):
            calls.append(number)
            if len(calls) == 2:
                api.pr['body'] = 'Fresh human edit\nResolves #1'
            return original(repo, number)
        api.linked = linked
        r.synchronize(self.root, {'SW-001': {}}, 'a/b', api, True)
        self.assertTrue(api.pr['body'].startswith('Fresh human edit\nResolves #1'))

    def test_strictdoc_export_with_real_grammar_and_links(self):
        import subprocess
        import shutil
        self.initialize()
        self.add('stakeholder', req('STK-001'))
        self.add('software', req('SW-001', parent='STK-001', category='Functional'))
        result = subprocess.run([shutil.which('strictdoc'), 'export', 'docs/requirements',
            '--config', 'strictdoc_config.py', '--output-dir', 'build/html', '--no-parallelization'],
            cwd=self.root, text=True, capture_output=True)
        self.assertEqual(0, result.returncode, result.stdout + result.stderr)
        self.assertTrue(list((self.root/'build/html').rglob('*.html')))

    def test_check_rejects_unknown_field_without_generating_html(self):
        import shutil
        self.initialize(['software'])
        self.add('software', req('SW-001', category='Functional'))
        binary = shutil.which('strictdoc')
        r.strictdoc_check(self.root, binary)
        self.assertEqual([], list(self.root.rglob('*.html')))
        self.add('software', 'UNKNOWNFIELD: x\n')
        with self.assertRaises(ValueError): r.strictdoc_check(self.root, binary)

    def test_off_runs_without_strictdoc_and_without_changes(self):
        import subprocess
        (self.root / 'heven.toml').write_text('[requirements]\nenabled=false\nlevels=[]\n')
        import sys
        result = subprocess.run([sys.executable, str(r.ROOT/'tools/requirements.py'), 'sync', '--root', str(self.root), '--apply'], capture_output=True)
        self.assertEqual(0, result.returncode)
        self.assertEqual(['heven.toml'], [p.name for p in self.root.iterdir()])

if __name__ == '__main__': unittest.main()
