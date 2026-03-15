from huggingface_hub import snapshot_download

snapshot_download(
    repo_id="Antii1/MITO_Dataset",
    repo_type="dataset",
    local_dir="MITO"
)