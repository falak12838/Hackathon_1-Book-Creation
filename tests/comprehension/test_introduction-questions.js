// Comprehension questions test for introduction chapter
// File: tests/comprehension/test_introduction-questions.js

const fs = require('fs');
const path = require('path');

function validateComprehensionQuestions() {
    const filePath = path.join(__dirname, '../../docs/robotic-nervous-system/introduction-to-ros2.md');

    if (!fs.existsSync(filePath)) {
        console.error('FAIL: Introduction chapter file does not exist');
        return false;
    }

    const content = fs.readFileSync(filePath, 'utf8');

    // Check if the file contains comprehension questions
    if (!content.toLowerCase().includes('question') && !content.toLowerCase().includes('exercise') && !content.toLowerCase().includes('quiz')) {
        console.error('FAIL: Introduction chapter does not contain comprehension questions or exercises');
        return false;
    }

    console.log('PASS: Introduction chapter comprehension questions validation');
    return true;
}

// Run the test
const result = validateComprehensionQuestions();
process.exit(result ? 0 : 1);