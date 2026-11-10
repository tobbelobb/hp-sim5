"""Conversation policies shared by local and current-chat managers."""
import json
from pathlib import Path


def manager_policy(*, grill_me=False, ask_questions=False):
    return '\nCommunication settings: ' + json.dumps({
        'grill_me': grill_me, 'ask_questions': ask_questions}) + '\n'


def worker_policy(session):
    return (
        '\nBlocking decision? Write {"id":"...","question":"... and your recommendation"} to '
        + str(session / 'manager-question.json') +
        ' and poll ' + str(session / 'manager-answer.json') +
        ' for the same id. Do not ask the human. Keep working meanwhile; '
        'if unanswered, report blocked or partial evidence.\n'
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
