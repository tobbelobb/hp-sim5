"""Conversation policies shared by local and current-chat managers."""
import json
from pathlib import Path


def manager_policy(*, grill_me=False, ask_questions=False):
    return '\nCommunication settings: ' + json.dumps({
        'grill_me': grill_me, 'ask_questions': ask_questions}) + '\n'


def worker_policy(session):
    return (
        '\nYour clarification contact is the research manager, never the human. '
        'Do not use human question tools. Resolve repository facts yourself. For a blocking '
        'manager decision, atomically write ' + str(session / 'manager-question.json') +
        ' as {"id":"a unique question id","question":"question, context and recommended answer"}. '
        'The manager polls worker_status and atomically writes ' + str(session / 'manager-answer.json') +
        ' as {"id":"the same question id","answer":"manager decision"}. '
        'Accept only an answer matching your current id. Ask one question at a time; keep '
        'unique ids so old answers cannot answer new questions. Continue independent work '
        'while waiting; poll at intervals of a few seconds, within the existing wall deadline. '
        'Never extend budgets or assume silence grants authority. If no answer arrives, '
        'retain the question and report blocked/partial evidence. Record answers in research.md.\n'
    )


def pending_question(session):
    """Read a small worker mailbox without ingesting logs or partial writes."""
    def read(name):
        with (Path(session) / name).open('rb') as stream:
            raw = stream.read(16385)
        if len(raw) > 16384:
            raise ValueError('Clarification exceeds 16 KiB')
        return json.loads(raw)

    try:
        question = read('manager-question.json')
        if not isinstance(question, dict) or not all(
                isinstance(question.get(key), str) and question[key].strip() for key in ('id', 'question')):
            return None
        try:
            answer = read('manager-answer.json')
        except (OSError, ValueError):
            answer = None
        if (isinstance(answer, dict) and answer.get('id') == question['id'] and
                isinstance(answer.get('answer'), str) and answer['answer'].strip()):
            return None
        return question
    except (OSError, ValueError):
        return None
