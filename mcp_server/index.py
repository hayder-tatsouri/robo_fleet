from server import mcp
from tools.navigation import navigate_to_pose


def main():
    mcp.run(transport="stdio")

if __name__ == "__main__":
    main()