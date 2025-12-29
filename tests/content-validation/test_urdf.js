// Content validation test for URDF chapter
// File: tests/content-validation/test_urdf.js

const fs = require('fs');
const path = require('path');

function validateURDFContent() {
    const filePath = path.join(__dirname, '../../docs/robotic-nervous-system/robot-structure-urdf.md');

    if (!fs.existsSync(filePath)) {
        console.error('FAIL: URDF chapter file does not exist');
        return false;
    }

    const content = fs.readFileSync(filePath, 'utf8');

    // Check if the file has basic structure
    if (!content.includes('#')) {
        console.error('FAIL: URDF chapter does not contain proper headings');
        return false;
    }

    // Check for basic content requirements
    const requiredKeywords = ['URDF', 'robot', 'structure', 'link', 'joint'];
    for (const keyword of requiredKeywords) {
        if (!content.toLowerCase().includes(keyword.toLowerCase())) {
            console.error(`FAIL: URDF chapter missing keyword: ${keyword}`);
            return false;
        }
    }

    console.log('PASS: URDF chapter content validation');
    return true;
}

// Run the test
const result = validateURDFContent();
process.exit(result ? 0 : 1);