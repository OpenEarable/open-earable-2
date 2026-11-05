#!/usr/bin/env python3
"""Check review rules with the same pinned CodeChecker engine used by CI."""
from pathlib import Path
from types import SimpleNamespace
import json,subprocess,sys,tempfile
from codechecker_common.review_status_handler import ReviewStatusHandler
ROOT=Path(__file__).resolve().parents[3]
config=ROOT/'.codechecker-review.json';rules=json.loads(config.read_text())['rules']
handler=ReviewStatusHandler();handler.set_review_status_config(str(config))
assert len(rules)==14
for rule in rules:
    filt=rule['filters'];assert set(filt)=={'filepath','checker_name','report_hash'}
    assert len(filt['report_hash'])==32 and filt['checker_name']=='core.FixedAddressDereference'
    assert rule['actions']['review_status']=='intentional' and rule['actions']['reason']
    path='/workspace'+filt['filepath'][1:]
    def report(path=path,checker=filt['checker_name'],bug_hash=filt['report_hash']):
        return SimpleNamespace(file=SimpleNamespace(original_path=path),checker_name=checker,report_hash=bug_hash)
    status=handler.get_review_status_from_config(report());assert status and status.status=='intentional'
    assert handler.get_review_status_from_config(report(bug_hash='0'*32)) is None
    assert handler.get_review_status_from_config(report(checker='security.ArrayBound')) is None
    assert handler.get_review_status_from_config(report(path='/workspace/application/unrelated.c')) is None
print('PASS: only the 14 reviewed checker/path/hash combinations are classified intentional')
with tempfile.TemporaryDirectory(prefix='oe-codechecker-review-') as temp:
    temp=Path(temp);active=temp/'active.json';reviewed=temp/'reviewed.json';output=temp/'comment.md'
    active.write_text(json.dumps({'reports':[]}));reviewed.write_text(json.dumps({'reports':[{'review_status':'intentional'}]}))
    cmd=[sys.executable,str(ROOT/'scripts/ci/codechecker_pr_report.py'),'--repo-root',str(ROOT),'--report',str(active),'--reviewed-report',str(reviewed),'--comment-output',str(output)]
    subprocess.run(cmd,check=True);body=output.read_text();assert 'No non-style issues found' in body and 'known nRF5340 hardware-register accesses' in body
    reviewed.write_text(json.dumps({'reports':[{'review_status':'unreviewed'}]}))
    result=subprocess.run(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE);assert result.returncode!=0
print('PASS: PR output discloses reviewed accesses and rejects unreviewed entries in the reviewed inventory')
