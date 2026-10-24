"""Program-bound manager MCP proxy. Worker physics MCP is separate."""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/python'))
from hp_sim5_research.program_host import read_json, request


def build_server(connection):
    # Optional SDK imports stay out of the ordinary-chat CLI route.
    from mcp.server import MCPServer
    from mcp_types import ToolAnnotations

    descriptor = read_json(connection)
    server = MCPServer('hp-sim5-program', instructions=(
        'Manager orchestration for one bound program. Start with program_overview and '
        'program_capabilities. Use program_call for the documented operations. No physics world is started.'))

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True, openWorldHint=False))
    def program_overview() -> dict:
        """Get durable program, evidence/worker paths, next allocation and current ownership."""
        return request(descriptor, 'overview')

    @server.tool(annotations=ToolAnnotations(readOnlyHint=True, openWorldHint=False))
    def program_capabilities() -> dict:
        """List actual operation parameters and explicit later-task limitations."""
        return request(descriptor, 'capabilities')

    @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=True, openWorldHint=False))
    def program_call(operation: str, parameters: dict) -> dict:
        """Call one manager operation. program_id cannot override the bound attachment."""
        return request(descriptor, operation, parameters)

    return server


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('connection', type=Path)
    args = parser.parse_args()
    build_server(args.connection).run(transport='stdio')
