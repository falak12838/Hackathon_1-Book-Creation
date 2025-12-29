import React from 'react';
import CodeBlock from '@theme/CodeBlock';

export default function CppExample({children, title}) {
  return (
    <div className="cpp-example">
      <h4>{title || 'C++ Example'}</h4>
      <CodeBlock language="cpp">
        {children}
      </CodeBlock>
    </div>
  );
}