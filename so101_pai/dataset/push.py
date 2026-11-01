"""Upload a local LeRobot dataset to the Hugging Face Hub as a PRIVATE dataset (a backup; training reads it locally).

    python -m so101_pai.dataset.push --repo-id <hf_user>/pai_sim --root data/datasets/pai_sim --yes

Needs `hf auth login` first. Without --yes it only prints what it would upload.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--yes", action="store_true", help="actually upload (otherwise dry run)")
    args = parser.parse_args()

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(args.repo_id, root=args.root)
    print(f"{args.root}: {dataset.meta.total_episodes} episodes, {dataset.meta.total_frames} frames")
    if not args.yes:
        print(f"dry run: add --yes to upload to https://huggingface.co/datasets/{args.repo_id} (private)")
        return
    dataset.push_to_hub(private=True, tags=["so101", "isaac-sim", "pai-final"])
    print(f"uploaded (private): https://huggingface.co/datasets/{args.repo_id}")


if __name__ == "__main__":
    main()
