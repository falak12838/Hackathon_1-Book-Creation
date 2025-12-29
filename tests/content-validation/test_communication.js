// Content validation test for communication chapter
// File: tests/content-validation/test_communication.js

const fs = require('fs');
const path = require('path');

function validateCommunicationContent() {
    const filePath = path.join(__dirname, '../../docs/robotic-nervous-system/communication-model.md');

    if (!fs.existsSync(filePath)) {
        console.error('FAIL: Communication chapter file does not exist');
        return false;
    }

    const content = fs.readFileSync(filePath, 'utf8');

    // Check if the file has basic structure
    if (!content.includes('#')) {
        console.error('FAIL: Communication chapter does not contain proper headings');
        return false;
    }

    // Check for basic content requirements
    const requiredKeywords = ['nodes', 'topics', 'services', 'communication'];
    for (const keyword of requiredKeywords) {
        if (!content.toLowerCase().includes(keyword.toLowerCase())) {
            console.error(`FAIL: Communication chapter missing keyword: ${keyword}`);
            return false;
        }
    }

    console.log('PASS: Communication chapter content validation');
    return true;
}

// Run the test
const result = validateCommunicationContent();
process.exit(result ? 0 : 1);