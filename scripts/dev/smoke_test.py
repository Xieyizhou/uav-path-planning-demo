"""Minimal development smoke test for local dependencies."""

from mavsdk import System


def main() -> None:
    _ = System
    print("MAVSDK import success")


if __name__ == "__main__":
    main()
