import os
import argparse
import json
from typing import List, Dict, Any
from langchain_text_splitters import TokenTextSplitter
import tiktoken
from models import Chunk # Assuming models.py is in the same directory

# A simple markdown parser to extract text and headings.
# For more robust parsing, a dedicated markdown library would be better.
def parse_markdown_for_text_and_headings(markdown_content: str) -> Dict[str, Any]:
    """
    Extracts clean text and attempts to find a primary heading from markdown.
    """
    lines = markdown_content.split('\n')
    clean_text_lines = []
    heading = ""
    
    for line in lines:
        stripped_line = line.strip()
        if stripped_line.startswith('# '): # Simple H1 heading detection
            if not heading: # Capture the first H1 as the primary heading
                heading = stripped_line[2:].strip()
            clean_text_lines.append(stripped_line) # Keep headings in text for chunking context
        elif stripped_line and not stripped_line.startswith(('!', '[', '>', '|', '---')): # Exclude images, links, blockquotes, tables, horizontal rules
            clean_text_lines.append(stripped_line)

    clean_text = "\n".join(clean_text_lines)
    return {"clean_text": clean_text, "heading": heading}


def extract_and_chunk_markdown_documents(docs_dir: str) -> List[Chunk]:
    """
    Recursively reads all markdown files, extracts clean text,
    splits them into chunks, and extracts metadata.
    """
    all_chunks: List[Chunk] = []
    text_splitter = TokenTextSplitter(chunk_size=800, chunk_overlap=160) # 20% of 800
    
    # Initialize tokenizer directly for counting tokens
    try:
        tokenizer = tiktoken.get_encoding("cl100k_base")
    except Exception:
        # Fallback if cl100k_base not found (though it should be standard)
        tokenizer = tiktoken.get_encoding("gpt2")

    for root, _, files in os.walk(docs_dir):
        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                try:
                    with open(file_path, 'r', encoding='utf-8') as f:
                        raw_content = f.read()

                    parsed_data = parse_markdown_for_text_and_headings(raw_content)
                    clean_text = parsed_data["clean_text"]
                    main_heading = parsed_data["heading"]

                    # Derive chapter from directory structure
                    # Assuming docs_dir is like 'Textbook/docs' and file_path is 'Textbook/docs/chapter1/intro.md'
                    relative_path = os.path.relpath(file_path, docs_dir)
                    chapter_name = os.path.dirname(relative_path).replace(os.sep, '_') # Use folder name as chapter
                    if not chapter_name: # For files directly in docs_dir
                        chapter_name = "General" 
                    
                    chunks_content = text_splitter.split_text(clean_text)

                    for i, chunk_content in enumerate(chunks_content):
                        # Create a temporary Chunk object to calculate hash and get token count
                        temp_chunk = Chunk(
                            id="", # Will be filled after hash calculation
                            content=chunk_content,
                            file_path=file_path,
                            heading=main_heading,
                            chapter=chapter_name,
                            token_count=len(tokenizer.encode(chunk_content)) # Use tiktoken tokenizer
                        )
                        # Hash the chunk content and metadata for unique ID
                        # Assuming calculate_content_hash is in utils.py
                        from utils import calculate_content_hash
                        temp_chunk.id = calculate_content_hash(temp_chunk)
                        all_chunks.append(temp_chunk)

                except Exception as e:
                    print(f"Error processing file {file_path}: {e}")
    return all_chunks

def main():
    parser = argparse.ArgumentParser(description="Extracts and chunks markdown content from Docusaurus docs.")
    parser.add_argument("--docs-dir", type=str, default="../../Textbook/docs",
                        help="Path to the Docusaurus docs directory.")
    parser.add_argument("--output", type=str, default="chunks.json",
                        help="Path to save the processed chunks JSON.")
    args = parser.parse_args()

    print(f"Starting markdown extraction and chunking from: {args.docs_dir}")
    chunks = extract_and_chunk_markdown_documents(args.docs_dir)
    print(f"Generated {len(chunks)} chunks.")

    # Convert Chunk dataclass objects to dictionaries for JSON serialization
    chunks_dict_list = [chunk.__dict__ for chunk in chunks]

    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(chunks_dict_list, f, indent=2)
    print(f"Processed chunks saved to {args.output}")


if __name__ == "__main__":
    main()
