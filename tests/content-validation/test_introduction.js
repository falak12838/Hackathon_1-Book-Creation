// Content validation test for introduction chapter
// File: tests/content-validation/test_introduction.js

const fs = require('fs');
const path = require('path');

function validateIntroductionContent() {
    const filePath = path.join(__dirname, '../../docs/robotic-nervous-system/introduction-to-ros2.md');

    if (!fs.existsSync(filePath)) {
        console.error('FAIL: Introduction chapter file does not exist');
        return false;
    }

    const content = fs.readFileSync(filePath, 'utf8');

    // Check if the file has basic structure
    if (!content.includes('#')) {
        console.error('FAIL: Introduction chapter does not contain proper headings');
        return false;
    }

    // Check for basic content requirements
    const requiredKeywords = ['ROS 2', 'humanoid', 'robotics'];
    for (const keyword of requiredKeywords) {
        if (!content.toLowerCase().includes(keyword.toLowerCase())) {
            console.error(`FAIL: Introduction chapter missing keyword: ${keyword}`);
            return false;
        }
    }

    console.log('PASS: Introduction chapter content validation');
    return true;
}

// Run the test
const result = validateIntroductionContent();
process.exit(result ? 0 : 1);