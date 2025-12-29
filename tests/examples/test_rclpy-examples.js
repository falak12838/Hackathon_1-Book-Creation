// rclpy examples validation test
// File: tests/examples/test_rclpy-examples.js

const fs = require('fs');
const path = require('path');

function validateRclpyExamples() {
    const filesToCheck = [
        path.join(__dirname, '../../docs/robotic-nervous-system/communication-model.md'),
        path.join(__dirname, '../../docs/robotic-nervous-system/nodes.md'),
        path.join(__dirname, '../../docs/robotic-nervous-system/topics.md'),
        path.join(__dirname, '../../docs/robotic-nervous-system/services.md')
    ];

    let foundRclpyExample = false;

    for (const filePath of filesToCheck) {
        if (!fs.existsSync(filePath)) {
            console.error(`FAIL: File does not exist: ${filePath}`);
            return false;
        }

        const content = fs.readFileSync(filePath, 'utf8');

        // Check for rclpy examples
        if (content.toLowerCase().includes('import rclpy') || content.includes('rclpy.init') || content.includes('rclpy.node')) {
            foundRclpyExample = true;
        }
    }

    if (!foundRclpyExample) {
        console.error('FAIL: No rclpy examples found in communication model files');
        return false;
    }

    console.log('PASS: rclpy examples validation');
    return true;
}

// Run the test
const result = validateRclpyExamples();
process.exit(result ? 0 : 1);