import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';
import CodeBlock from '@theme/CodeBlock';

export default function CodeExample({python, cpp, title, description}) {
  return (
    <div className="code-example">
      <h4>{title || 'Code Example'}</h4>
      {description && <p>{description}</p>}

      <Tabs>
        {python && (
          <TabItem value="python" label="Python">
            <CodeBlock language="python">
              {python}
            </CodeBlock>
          </TabItem>
        )}
        {cpp && (
          <TabItem value="cpp" label="C++">
            <CodeBlock language="cpp">
              {cpp}
            </CodeBlock>
          </TabItem>
        )}
      </Tabs>
    </div>
  );
}