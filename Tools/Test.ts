import * as fs from "fs";
import * as path from "path";
import * as child_process from "child_process"

const files = fs.readdirSync(path.join(__dirname, "../FBX/Source"));
const destDir = path.join(__dirname, "../FBX/Bin")

const processPath = path.join(__dirname, "../Build/MinSizeRel/FbxParser.exe");
const promises = files.map((file) => {
    const extraceAbsDir = path.join(destDir, file, "out");
    const logPath = path.join(destDir, file, 'out/fbx.log');
    const fbmPath = path.join(destDir, file, 'fbm');
    const sourceFile = path.join(__dirname, "../FBX/Source", file);
    const task = child_process.spawn(processPath, ["-o", extraceAbsDir, "-f", sourceFile, "-l", logPath, '-t', fbmPath], {
        stdio: 'pipe',
        shell: false,
        cwd: path.dirname(processPath),
    });
    return new Promise(async (resolve, reject) => {
        task.stdout.on('data', (data: Buffer) => {
            console.log(`[${sourceFile}]${data.toString()}`);
        });
        task.stderr.on('data', (data: Buffer) => {
            console.log(`[${sourceFile}]${data.toString()}`);
        });
        task.on('exit', (code: number) => {
            if (!task.killed) {
                task.kill();
            }
            resolve(null);
        });
        task.on('error', (err: Error) => {
            resolve(null);
        });
        task.on('disconnect', () => {
            resolve(null);
        });
    });
});

Promise.all(promises);