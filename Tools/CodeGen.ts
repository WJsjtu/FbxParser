import * as fs from "fs";
import * as path from "path";
import * as TJS from "typescript-json-schema";
import * as iconv from "iconv-lite";


/**
 * if you want non-ASCII characters then the "official" and portable way to get them is to use the \u (or \U) hex encoding (which is, I agree, just plain ugly and error prone).
 * The compiler when faced with a source file that does not have a BOM the compiler reads ahead a certain distance into the file to see if it can detect any Unicode characters - it specifically looks for UTF-16 and UTF-16BE - if it doesn't find either then it assumes that it has MBCS. I suspect that in this case that in this case it falls back to MBCS and this is what is causing the problem.
 * Being explicit is really best and so while I know it is not a perfect solution I would suggest using the BOM.
 * Jonathan Caves
 * Visual C++ Compiler Team.
 * @param str 
 */
function EncodeString(str: string): Buffer {
  return iconv.encode(str, "utf8", process.platform === "win32" ? {
    addBOM: true,
  } : undefined);
}

(function SchemaCode() {
  // optionally pass argument to schema generator
  const settings: TJS.PartialArgs = {
    required: true,
  };

  // optionally pass ts compiler options
  const compilerOptions: TJS.CompilerOptions = {
    strictNullChecks: true,
    moduleResolution: 2
  }

  // optionally pass a base path
  const basePath = "./";
  const program = TJS.getProgramFromFiles([path.resolve(__dirname, "ImportOptions.ts")], compilerOptions, basePath);
  const schema = TJS.generateSchema(program, "IImportOptions", settings);
  let schemaString = JSON.stringify(JSON.stringify(schema));
  schemaString = "static const std::string ImportOptionsJsonSchema = " + schemaString + ";\n\n"
  const code = "#pragma once\n#include <iostream>\n#include <unordered_map>\n#include <vector>\n#include <string>\n" + schemaString + "\n"
  fs.writeFileSync(path.resolve(__dirname, "../FBX/ImportOptions.hpp"), EncodeString(code));
})();