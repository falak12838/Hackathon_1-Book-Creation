import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import CodeBlock from '@theme/CodeBlock';

export default function PythonExample({children, title}) {
  return (
    <div className="python-example">
      <h4>{title || 'Python Example'}</h4>
      <CodeBlock language="python">
        {children}
      </CodeBlock>
    </div>
  );
}