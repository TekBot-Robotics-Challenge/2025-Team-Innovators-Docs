import { useState, useRef } from 'react';
import { Copy, Check } from 'lucide-react';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { oneDark } from 'react-syntax-highlighter/dist/cjs/styles/prism';

type CodeViewerProps = {
  code: string;
  language: string;
  className?: string;
  showLineNumbers?: boolean;
  startingLineNumber?: number;
  wrapLines?: boolean;
};

export const CodeViewer = ({
  code,
  language,
  className = '',
  showLineNumbers = true,
  startingLineNumber = 1,
  wrapLines = false,
}: CodeViewerProps) => {
  const [copied, setCopied] = useState(false);
  const codeRef = useRef<HTMLDivElement>(null);

  const handleCopy = () => {
    navigator.clipboard.writeText(code);
    setCopied(true);
    setTimeout(() => setCopied(false), 2000);
  };

  return (
    <div
      className={`relative rounded-lg bg-gray-900 overflow-hidden border border-gray-700 group ${className}`}
      ref={codeRef}
    >
      <div className="flex justify-between items-center bg-gray-800 px-4 py-2">
        <span className="text-xs text-gray-400">{language}</span>
        <button
          onClick={handleCopy}
          className="flex items-center gap-1 text-xs text-gray-400 hover:text-gray-200 transition-colors"
          aria-label="Copier le code"
        >
          {copied ? (
            <>
              <Check className="w-4 h-4 text-green-400" />
              <span>Copié!</span>
            </>
          ) : (
            <>
              <Copy className="w-4 h-4" />
              <span>Copier</span>
            </>
          )}
        </button>
      </div>

      <SyntaxHighlighter
        language={language}
        style={oneDark}
        showLineNumbers={showLineNumbers}
        startingLineNumber={startingLineNumber}
        wrapLines={wrapLines}
        wrapLongLines={wrapLines}
        lineNumberStyle={{ color: '#6b7280', minWidth: '2.5em' }}
        customStyle={{
          margin: 0,
          padding: '1rem',
          background: '#111827',
          fontSize: '0.875rem',
          lineHeight: '1.5',
          borderRadius: '0 0 0.5rem 0.5rem',
        }}
        codeTagProps={{
          style: {
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Monaco, Consolas, "Liberation Mono", "Courier New", monospace',
          },
        }}
      >
        {code}
      </SyntaxHighlighter>

      {/* Gradient overlay pour le hover */}
      <div className="absolute inset-0 pointer-events-none opacity-0 group-hover:opacity-100 transition-opacity bg-gradient-to-b from-gray-900/10 to-gray-900/30" />
    </div>
  );
};