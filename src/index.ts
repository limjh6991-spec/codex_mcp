#!/usr/bin/env node

import { Server } from "@modelcontextprotocol/sdk/server/index.js";
import { StdioServerTransport } from "@modelcontextprotocol/sdk/server/stdio.js";
import {
  CallToolRequestSchema,
  ListToolsRequestSchema,
} from "@modelcontextprotocol/sdk/types.js";
import * as fs from "fs/promises";
import * as path from "path";

/**
 * Codex MCP Server
 * Provides code analysis and manipulation tools for VS Code integration
 */
class CodexMCPServer {
  private server: Server;

  constructor() {
    this.server = new Server(
      {
        name: "codex-mcp-server",
        version: "1.0.0",
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupToolHandlers();
    
    // Error handling
    this.server.onerror = (error) => console.error("[MCP Error]", error);
    process.on("SIGINT", async () => {
      await this.server.close();
      process.exit(0);
    });
  }

  private setupToolHandlers() {
    // List available tools
    this.server.setRequestHandler(ListToolsRequestSchema, async () => ({
      tools: [
        {
          name: "read_file",
          description: "Read the contents of a file from the workspace",
          inputSchema: {
            type: "object",
            properties: {
              path: {
                type: "string",
                description: "Path to the file to read (relative or absolute)",
              },
            },
            required: ["path"],
          },
        },
        {
          name: "write_file",
          description: "Write content to a file in the workspace",
          inputSchema: {
            type: "object",
            properties: {
              path: {
                type: "string",
                description: "Path to the file to write (relative or absolute)",
              },
              content: {
                type: "string",
                description: "Content to write to the file",
              },
            },
            required: ["path", "content"],
          },
        },
        {
          name: "list_directory",
          description: "List files and directories in a given path",
          inputSchema: {
            type: "object",
            properties: {
              path: {
                type: "string",
                description: "Path to the directory to list",
              },
            },
            required: ["path"],
          },
        },
        {
          name: "analyze_code",
          description: "Analyze code structure and provide insights (placeholder for AI analysis)",
          inputSchema: {
            type: "object",
            properties: {
              code: {
                type: "string",
                description: "Code to analyze",
              },
              language: {
                type: "string",
                description: "Programming language of the code",
              },
            },
            required: ["code"],
          },
        },
        {
          name: "search_files",
          description: "Search for files matching a pattern in a directory",
          inputSchema: {
            type: "object",
            properties: {
              directory: {
                type: "string",
                description: "Directory to search in",
              },
              pattern: {
                type: "string",
                description: "File name pattern to match (supports wildcards)",
              },
            },
            required: ["directory", "pattern"],
          },
        },
      ],
    }));

    // Handle tool calls
    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const { name, arguments: args } = request.params;

      try {
        switch (name) {
          case "read_file":
            return await this.handleReadFile(args);
          case "write_file":
            return await this.handleWriteFile(args);
          case "list_directory":
            return await this.handleListDirectory(args);
          case "analyze_code":
            return await this.handleAnalyzeCode(args);
          case "search_files":
            return await this.handleSearchFiles(args);
          default:
            throw new Error(`Unknown tool: ${name}`);
        }
      } catch (error) {
        const errorMessage = error instanceof Error ? error.message : String(error);
        return {
          content: [
            {
              type: "text",
              text: `Error: ${errorMessage}`,
            },
          ],
        };
      }
    });
  }

  private async handleReadFile(args: any) {
    const filePath = args.path as string;
    const content = await fs.readFile(filePath, "utf-8");
    return {
      content: [
        {
          type: "text",
          text: content,
        },
      ],
    };
  }

  private async handleWriteFile(args: any) {
    const filePath = args.path as string;
    const content = args.content as string;
    
    // Ensure directory exists
    const dir = path.dirname(filePath);
    await fs.mkdir(dir, { recursive: true });
    
    await fs.writeFile(filePath, content, "utf-8");
    return {
      content: [
        {
          type: "text",
          text: `Successfully wrote to ${filePath}`,
        },
      ],
    };
  }

  private async handleListDirectory(args: any) {
    const dirPath = args.path as string;
    const entries = await fs.readdir(dirPath, { withFileTypes: true });
    
    const items = entries.map(entry => ({
      name: entry.name,
      type: entry.isDirectory() ? "directory" : "file",
    }));
    
    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(items, null, 2),
        },
      ],
    };
  }

  private async handleAnalyzeCode(args: any) {
    const code = args.code as string;
    const language = args.language as string || "unknown";
    
    // Basic code analysis
    const lines = code.split("\n").length;
    const characters = code.length;
    const hasClasses = /class\s+\w+/.test(code);
    const hasFunctions = /function\s+\w+|const\s+\w+\s*=\s*\(|\w+\s*\(.*\)\s*=>/.test(code);
    const hasComments = /\/\/|\/\*|\*\/|#/.test(code);
    
    const analysis = {
      language,
      statistics: {
        lines,
        characters,
      },
      features: {
        hasClasses,
        hasFunctions,
        hasComments,
      },
      suggestion: "This is a basic analysis. For advanced AI-powered analysis, integrate with OpenAI Codex or similar services.",
    };
    
    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(analysis, null, 2),
        },
      ],
    };
  }

  private async handleSearchFiles(args: any) {
    const directory = args.directory as string;
    const pattern = args.pattern as string;
    
    const results: string[] = [];
    
    async function searchRecursive(dir: string) {
      const entries = await fs.readdir(dir, { withFileTypes: true });
      
      for (const entry of entries) {
        const fullPath = path.join(dir, entry.name);
        
        if (entry.isDirectory()) {
          await searchRecursive(fullPath);
        } else if (entry.isFile()) {
          // Simple pattern matching (convert * to .*)
          const regex = new RegExp(pattern.replace(/\*/g, ".*"));
          if (regex.test(entry.name)) {
            results.push(fullPath);
          }
        }
      }
    }
    
    await searchRecursive(directory);
    
    return {
      content: [
        {
          type: "text",
          text: JSON.stringify(results, null, 2),
        },
      ],
    };
  }

  async run() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error("Codex MCP Server running on stdio");
  }
}

// Start the server
const server = new CodexMCPServer();
server.run().catch(console.error);
