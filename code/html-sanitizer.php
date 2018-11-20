<?php

require __DIR__.'/../vendor/autoload.php';

$sanitizer = \HtmlSanitizer\Sanitizer::create(['extensions' => ['basic', 'code', 'image', 'list', 'table', 'extra']]);

$input = file_get_contents(__DIR__.'/fixture.html');
$times = 100;
$time = microtime(true);

echo "Running...\n";

for ($i = 0; $i < $times; $i++) {
    $output = $sanitizer->sanitize($input);
}

$total = (microtime(true) - $time) * 1000;

echo 'Total for '.$times.' loops: '.round($total, 2)."ms\n";
echo 'Time per loop: '.round($total / $times, 2)."ms\n";
echo "\n";

<?xml version="1.0" encoding="UTF-8"?>

<!-- https://phpunit.de/manual/current/en/appendixes.configuration.html -->
<phpunit xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
         xsi:noNamespaceSchemaLocation="http://schema.phpunit.de/6.1/phpunit.xsd"
         backupGlobals="false"
         colors="true"
         bootstrap="vendor/autoload.php">
    <php>
        <ini name="error_reporting" value="-1" />
        <env name="SHELL_VERBOSITY" value="-1" />
    </php>

    <testsuites>
        <testsuite name="HtmlSanitizer Test Suite">
            <directory>tests/</directory>
        </testsuite>
    </testsuites>

    <filter>
        <whitelist>
            <directory>./src/</directory>
        </whitelist>
    </filter>
</phpunit>

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\DocumentNode;
use HtmlSanitizer\Node\TextNode;
use HtmlSanitizer\Visitor\NodeVisitorInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DomVisitor implements DomVisitorInterface
{
    /**
     * @var NodeVisitorInterface[]
     */
    private $visitors;

    /**
     * @var NodeVisitorInterface[]
     */
    private $reversedVisitors;

    public function __construct(array $visitors = [])
    {
        $this->visitors = $visitors;
    }

    public function visit(\DOMNode $node): DocumentNode
    {
        if (!$this->reversedVisitors) {
            $this->reversedVisitors = array_reverse($this->visitors);
        }

        $cursor = new Cursor();
        $cursor->node = new DocumentNode();

        $this->visitNode($node, $cursor);

        return $cursor->node;
    }

    private function visitNode(\DOMNode $node, Cursor $cursor)
    {
        foreach ($this->visitors as $visitor) {
            if ($visitor->supports($node, $cursor)) {
                $visitor->enterNode($node, $cursor);
            }
        }

        foreach ($node->childNodes ?? [] as $k => $child) {
            if ('#text' === $child->nodeName) {
                // Add text in the safe tree without a visitor for performance
                $cursor->node->addChild(new TextNode($cursor->node, $child->nodeValue));
            } elseif (!$child instanceof \DOMText) { // Ignore HTML comments
                $this->visitNode($child, $cursor);
            }
        }

        foreach ($this->reversedVisitors as $visitor) {
            if ($visitor->supports($node, $cursor)) {
                $visitor->leaveNode($node, $cursor);
            }
        }
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer;

use HtmlSanitizer\Node\DocumentNode;

/**
 * Visit a parsed DOM node to create the equivalent sanitized DocumentNode.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface DomVisitorInterface
{
    public function visit(\DOMNode $node): DocumentNode;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Exception;

use HtmlSanitizer\Parser\ParserInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
class ParsingFailedException extends \InvalidArgumentException
{
    /**
     * @var ParserInterface
     */
    private $parser;

    public function __construct(ParserInterface $parser, \Throwable $previous = null)
    {
        parent::__construct('HTML parsing failed using parser '.get_class($parser), 0, $previous);

        $this->parser = $parser;
    }

    public function getParser(): ParserInterface
    {
        return $this->parser;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class BasicExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'basic';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'a' => new NodeVisitor\ANodeVisitor($config['tags']['a'] ?? []),
            'blockquote' => new NodeVisitor\BlockquoteNodeVisitor($config['tags']['blockquote'] ?? []),
            'br' => new NodeVisitor\BrNodeVisitor($config['tags']['br'] ?? []),
            'div' => new NodeVisitor\DivNodeVisitor($config['tags']['div'] ?? []),
            'del' => new NodeVisitor\DelNodeVisitor($config['tags']['del'] ?? []),
            'em' => new NodeVisitor\EmNodeVisitor($config['tags']['em'] ?? []),
            'figcaption' => new NodeVisitor\FigcaptionNodeVisitor($config['tags']['figcaption'] ?? []),
            'figure' => new NodeVisitor\FigureNodeVisitor($config['tags']['figure'] ?? []),
            'h1' => new NodeVisitor\H1NodeVisitor($config['tags']['h1'] ?? []),
            'h2' => new NodeVisitor\H2NodeVisitor($config['tags']['h2'] ?? []),
            'h3' => new NodeVisitor\H3NodeVisitor($config['tags']['h3'] ?? []),
            'h4' => new NodeVisitor\H4NodeVisitor($config['tags']['h4'] ?? []),
            'h5' => new NodeVisitor\H5NodeVisitor($config['tags']['h5'] ?? []),
            'h6' => new NodeVisitor\H6NodeVisitor($config['tags']['h6'] ?? []),
            'i' => new NodeVisitor\INodeVisitor($config['tags']['i'] ?? []),
            'p' => new NodeVisitor\PNodeVisitor($config['tags']['p'] ?? []),
            'q' => new NodeVisitor\QNodeVisitor($config['tags']['q'] ?? []),
            'small' => new NodeVisitor\SmallNodeVisitor($config['tags']['small'] ?? []),
            'span' => new NodeVisitor\SpanNodeVisitor($config['tags']['span'] ?? []),
            'strong' => new NodeVisitor\StrongNodeVisitor($config['tags']['strong'] ?? []),
            'sub' => new NodeVisitor\SubNodeVisitor($config['tags']['sub'] ?? []),
            'sup' => new NodeVisitor\SupNodeVisitor($config['tags']['sup'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ANode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'a';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class BlockquoteNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'blockquote';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\IsChildlessTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class BrNode extends AbstractTagNode
{
    use IsChildlessTrait;

    public function getTagName(): string
    {
        return 'br';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DelNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'del';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DivNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'div';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class EmNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'em';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class FigcaptionNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'figcaption';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class FigureNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'figure';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H1Node extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'h1';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H2Node extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'h2';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H3Node extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'h3';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H4Node extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'h4';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H5Node extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'h5';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H6Node extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'h6';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class INode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'i';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class PNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'p';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class QNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'q';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SmallNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'small';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SpanNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'span';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class StrongNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'strong';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SubNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'sub';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SupNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'sup';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\ANode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Sanitizer\AHrefSanitizer;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ANodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    private $sanitizer;

    public function __construct(array $config = [])
    {
        parent::__construct($config);

        $this->sanitizer = new AHrefSanitizer(
            $this->config['allowed_hosts'],
            $this->config['allow_mailto'],
            $this->config['force_https']
        );
    }

    protected function getDomNodeName(): string
    {
        return 'a';
    }

    public function getDefaultAllowedAttributes(): array
    {
        return ['href', 'title'];
    }

    public function getDefaultConfiguration(): array
    {
        return [
            'allowed_hosts' => null,
            'allow_mailto' => true,
            'force_https' => false,
        ];
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        $node = new ANode($cursor->node);
        $node->setAttribute('href', $this->sanitizer->sanitize($this->getAttribute($domNode, 'href')));

        return $node;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\BlockquoteNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class BlockquoteNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'blockquote';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new BlockquoteNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\BrNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\IsChildlessTagVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class BrNodeVisitor extends AbstractNodeVisitor
{
    use IsChildlessTagVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'br';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new BrNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\DelNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DelNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'del';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new DelNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\DivNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DivNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'div';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new DivNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\EmNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class EmNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'em';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new EmNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\FigcaptionNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class FigcaptionNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'figcaption';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new FigcaptionNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\FigureNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class FigureNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'figure';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new FigureNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\H1Node;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H1NodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'h1';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new H1Node($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\H2Node;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H2NodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'h2';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new H2Node($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\H3Node;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H3NodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'h3';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new H3Node($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\H4Node;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H4NodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'h4';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new H4Node($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\H5Node;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H5NodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'h5';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new H5Node($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\H6Node;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class H6NodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'h6';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new H6Node($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Basic\Node\INode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class INodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'i';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new INode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\PNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class PNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'p';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new PNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\QNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class QNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'q';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new QNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\SmallNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SmallNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'small';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new SmallNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\SpanNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SpanNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'span';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new SpanNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\StrongNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class StrongNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'strong';
    }

    public function supports(\DOMNode $domNode, Cursor $cursor): bool
    {
        return 'strong' === $domNode->nodeName || 'b' === $domNode->nodeName;
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new StrongNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\SubNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SubNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'sub';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new SubNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Basic\Node\SupNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SupNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'sup';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new SupNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Basic\Sanitizer;

use HtmlSanitizer\Sanitizer\UrlSanitizerTrait;

/**
 * @internal
 */
class AHrefSanitizer
{
    use UrlSanitizerTrait;

    private $allowedHosts;
    private $allowMailTo;
    private $forceHttps;

    public function __construct(?array $allowedHosts, bool $allowMailTo, bool $forceHttps)
    {
        $this->allowedHosts = $allowedHosts;
        $this->allowMailTo = $allowMailTo;
        $this->forceHttps = $forceHttps;
    }

    public function sanitize(?string $input): ?string
    {
        $allowedSchemes = ['http', 'https'];
        $allowedHosts = $this->allowedHosts;

        if ($this->allowMailTo) {
            $allowedSchemes[] = 'mailto';

            if (\is_array($this->allowedHosts)) {
                $allowedHosts[] = null;
            }
        }

        $sanitized = $this->sanitizeUrl($input, $allowedSchemes, $allowedHosts, $this->forceHttps);

        // Basic validation that it's an e-mail
        if (strpos($sanitized, 'mailto:') === 0 && (strpos($sanitized, '@') === false || strpos($sanitized, '.') === false)) {
            return null;
        }

        return $sanitized;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Code;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class CodeExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'code';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'code' => new NodeVisitor\CodeNodeVisitor($config['tags']['code'] ?? []),
            'pre' => new NodeVisitor\PreNodeVisitor($config['tags']['pre'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Code\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class CodeNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'code';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Code\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class PreNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'pre';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Code\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Code\Node\CodeNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class CodeNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'code';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new CodeNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Code\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Code\Node\PreNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class PreNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'pre';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new PreNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension;

use HtmlSanitizer\Visitor\NodeVisitorInterface;

/**
 * A sanitizer extension allows to easily add features to the sanitizer to handle specific tags.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface ExtensionInterface
{
    /**
     * Return this extension name, which will be used in the sanitizer configuration.
     */
    public function getName(): string;

    /**
     * Return a list of node visitors to register in the sanitizer following the format tagName => visitor.
     * For instance:
     *
     *      'strong' => new StrongVisitor($config,
     *
     * @param array $config The configuration given by the user of the library.
     *
     * @return NodeVisitorInterface[]
     */
    public function createNodeVisitors(array $config = []): array;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ExtraExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'extra';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'abbr' => new NodeVisitor\AbbrNodeVisitor($config['tags']['abbr'] ?? []),
            'caption' => new NodeVisitor\CaptionNodeVisitor($config['tags']['caption'] ?? []),
            'hr' => new NodeVisitor\HrNodeVisitor($config['tags']['hr'] ?? []),
            'rp' => new NodeVisitor\RpNodeVisitor($config['tags']['rp'] ?? []),
            'rt' => new NodeVisitor\RtNodeVisitor($config['tags']['rt'] ?? []),
            'ruby' => new NodeVisitor\RubyNodeVisitor($config['tags']['ruby'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class AbbrNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'abbr';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class CaptionNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'caption';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\IsChildlessTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class HrNode extends AbstractTagNode
{
    use IsChildlessTrait;

    public function getTagName(): string
    {
        return 'hr';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class RpNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'rp';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class RtNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'rt';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class RubyNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'ruby';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Extra\Node\AbbrNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class AbbrNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'abbr';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new AbbrNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Extra\Node\CaptionNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class CaptionNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'caption';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new CaptionNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Extra\Node\HrNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\IsChildlessTagVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class HrNodeVisitor extends AbstractNodeVisitor
{
    use IsChildlessTagVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'hr';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new HrNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Extra\Node\RpNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class RpNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'rp';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new RpNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Extra\Node\RtNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class RtNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'rt';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new RtNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Extra\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Extra\Node\RubyNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class RubyNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'ruby';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new RubyNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Iframe;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class IframeExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'iframe';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'iframe' => new NodeVisitor\IframeNodeVisitor($config['tags']['iframe'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Iframe\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class IframeNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'iframe';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Iframe\NodeVisitor;

use HtmlSanitizer\Extension\Iframe\Node\IframeNode;
use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Iframe\Sanitizer\IframeSrcSanitizer;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class IframeNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    /**
     * @var IframeSrcSanitizer
     */
    private $sanitizer;

    public function __construct(array $config = [])
    {
        parent::__construct($config);

        $this->sanitizer = new IframeSrcSanitizer($this->config['allowed_hosts'], $this->config['force_https']);
    }

    protected function getDomNodeName(): string
    {
        return 'iframe';
    }

    public function getDefaultAllowedAttributes(): array
    {
        return [
            'src', 'width', 'height', 'frameborder', 'title',

            // YouTube integration
            'allow', 'allowfullscreen',
        ];
    }

    public function getDefaultConfiguration(): array
    {
        return [
            'allowed_hosts' => null,
            'force_https' => false,
        ];
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        $node = new IframeNode($cursor->node);
        $node->setAttribute('src', $this->sanitizer->sanitize($this->getAttribute($domNode, 'src')));

        return $node;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Iframe\Sanitizer;

use HtmlSanitizer\Sanitizer\UrlSanitizerTrait;

/**
 * @internal
 */
class IframeSrcSanitizer
{
    use UrlSanitizerTrait;

    private $allowedHosts;
    private $forceHttps;

    public function __construct(?array $allowedHosts, bool $forceHttps)
    {
        $this->allowedHosts = $allowedHosts;
        $this->forceHttps = $forceHttps;
    }

    public function sanitize(?string $input): ?string
    {
        return $this->sanitizeUrl($input, ['http', 'https'], $this->allowedHosts, $this->forceHttps);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Image;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ImageExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'image';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'img' => new NodeVisitor\ImgNodeVisitor($config['tags']['img'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Image\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\IsChildlessTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ImgNode extends AbstractTagNode
{
    use IsChildlessTrait;

    public function getTagName(): string
    {
        return 'img';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Image\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Image\Node\ImgNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Image\Sanitizer\ImgSrcSanitizer;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\IsChildlessTagVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ImgNodeVisitor extends AbstractNodeVisitor
{
    use IsChildlessTagVisitorTrait;

    /**
     * @var ImgSrcSanitizer
     */
    private $sanitizer;

    public function __construct(array $config = [])
    {
        parent::__construct($config);

        $this->sanitizer = new ImgSrcSanitizer(
            $this->config['allowed_hosts'],
            $this->config['allow_data_uri'],
            $this->config['force_https']
        );
    }

    protected function getDomNodeName(): string
    {
        return 'img';
    }

    public function getDefaultAllowedAttributes(): array
    {
        return ['src', 'alt', 'title'];
    }

    public function getDefaultConfiguration(): array
    {
        return [
            'allowed_hosts' => null,
            'allow_data_uri' => false,
            'force_https' => false,
        ];
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        $node = new ImgNode($cursor->node);
        $node->setAttribute('src', $this->sanitizer->sanitize($this->getAttribute($domNode, 'src')));

        return $node;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Image\Sanitizer;

use HtmlSanitizer\Sanitizer\UrlSanitizerTrait;

/**
 * @internal
 */
class ImgSrcSanitizer
{
    use UrlSanitizerTrait;

    private $allowedHosts;
    private $allowDataUri;
    private $forceHttps;

    public function __construct(?array $allowedHosts, bool $allowDataUri, bool $forceHttps)
    {
        $this->allowedHosts = $allowedHosts;
        $this->allowDataUri = $allowDataUri;
        $this->forceHttps = $forceHttps;
    }

    public function sanitize(?string $input): ?string
    {
        $allowedSchemes = ['http', 'https'];
        $allowedHosts = $this->allowedHosts;

        if ($this->allowDataUri) {
            $allowedSchemes[] = 'data';
            $allowedHosts[] = null;
        }

        $sanitized = $this->sanitizeUrl($input, $allowedSchemes, $allowedHosts, $this->forceHttps);

        // Allow only images in data URIs
        if (strpos($sanitized, 'data:') === 0 && strpos($sanitized, 'data:image/') !== 0) {
            return null;
        }

        return $sanitized;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ListExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'list';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'dd' => new NodeVisitor\DdNodeVisitor($config['tags']['dd'] ?? []),
            'dl' => new NodeVisitor\DlNodeVisitor($config['tags']['dl'] ?? []),
            'dt' => new NodeVisitor\DtNodeVisitor($config['tags']['dt'] ?? []),
            'li' => new NodeVisitor\LiNodeVisitor($config['tags']['li'] ?? []),
            'ol' => new NodeVisitor\OlNodeVisitor($config['tags']['ol'] ?? []),
            'ul' => new NodeVisitor\UlNodeVisitor($config['tags']['ul'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DdNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'dd';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DlNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'dl';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DtNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'dt';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class LiNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'li';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class OlNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'ol';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class UlNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'ul';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Listing\Node\DdNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DdNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'dd';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new DdNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Listing\Node\DlNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DlNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'dl';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new DlNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Listing\Node\DtNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class DtNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'dt';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new DtNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Extension\Listing\Node\LiNode;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class LiNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'li';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new LiNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Listing\Node\OlNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class OlNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'ol';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new OlNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Listing\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Listing\Node\UlNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class UlNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'ul';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new UlNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TableNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'table';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TbodyNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'tbody';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TdNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'td';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TfootNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'tfoot';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TheadNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'thead';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ThNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'th';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TrNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'tr';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\TableNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TableNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'table';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new TableNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\TbodyNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TbodyNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'tbody';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new TbodyNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\TdNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TdNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'td';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new TdNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\TfootNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TfootNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'tfoot';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new TfootNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\TheadNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TheadNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'thead';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new TheadNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\ThNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class ThNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'th';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new ThNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Extension\Table\Node\TrNode;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TrNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'tr';
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        return new TrNode($cursor->node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Extension\Table;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class TableExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'table';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'table' => new NodeVisitor\TableNodeVisitor($config['tags']['table'] ?? []),
            'tbody' => new NodeVisitor\TbodyNodeVisitor($config['tags']['tbody'] ?? []),
            'td' => new NodeVisitor\TdNodeVisitor($config['tags']['td'] ?? []),
            'tfoot' => new NodeVisitor\TfootNodeVisitor($config['tags']['tfoot'] ?? []),
            'thead' => new NodeVisitor\TheadNodeVisitor($config['tags']['thead'] ?? []),
            'th' => new NodeVisitor\ThNodeVisitor($config['tags']['th'] ?? []),
            'tr' => new NodeVisitor\TrNodeVisitor($config['tags']['tr'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Model;

use HtmlSanitizer\Node\NodeInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class Cursor
{
    /**
     * @var NodeInterface
     */
    public $node;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
abstract class AbstractNode implements NodeInterface
{
    private $parent;

    public function __construct(NodeInterface $parent)
    {
        $this->parent = $parent;
    }

    public function getParent(): ?NodeInterface
    {
        return $this->parent;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

use HtmlSanitizer\Sanitizer\StringSanitizerTrait;

/**
 * Abstract base class for tags.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
abstract class AbstractTagNode extends AbstractNode implements TagNodeInterface
{
    use StringSanitizerTrait;

    private $attributes = [];

    /**
     * Return this tag name (used to render it).
     *
     * @return string
     */
    abstract public function getTagName(): string;

    public function getAttribute(string $name): ?string
    {
        return $this->attributes[$name] ?? null;
    }

    public function setAttribute(string $name, ?string $value)
    {
        // Always use only the first declaration (ease sanitization)
        if (!array_key_exists($name, $this->attributes)) {
            $this->attributes[$name] = $value;
        }
    }

    public function render(): string
    {
        $tag = $this->getTagName();

        if (method_exists($this, 'renderChildren')) {
            return '<'.$tag.$this->renderAttributes().'>'.$this->renderChildren().'</'.$tag.'>';
        }

        return '<'.$tag.$this->renderAttributes().' />';
    }

    protected function renderAttributes(): string
    {
        $rendered = [];
        foreach ($this->attributes as $name => $value) {
            if ($value === null) {
                // Tag should be removed as a sanitizer found suspect data inside
                continue;
            }

            $attr = $this->encodeHtmlEntities($name);
            if ($value !== '') {
                // In quirks mode, IE8 does a poor job producing innerHTML values.
                // If JavaScript does:
                //      nodeA.innerHTML = nodeB.innerHTML;
                // and nodeB contains (or even if ` was encoded properly):
                //      <div attr="``foo=bar">
                // then IE8 will produce:
                //      <div attr=``foo=bar>
                // as the value of nodeB.innerHTML and assign it to nodeA.
                // IE8's HTML parser treats `` as a blank attribute value and foo=bar becomes a separate attribute.
                // Adding a space at the end of the attribute prevents this by forcing IE8 to put double
                // quotes around the attribute when computing nodeB.innerHTML.
                if (false !== mb_strpos($value, '`')) {
                    $value .= ' ';
                }

                $attr .= '="'.$this->encodeHtmlEntities($value).'"';
            }

            $rendered[] = $attr;
        }

        return $rendered ? ' '.implode(' ', $rendered) : '';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Root node of the sanitized HTML. Contains all the other nodes.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class DocumentNode implements NodeInterface
{
    use HasChildrenTrait;

    public function getParent(): ?NodeInterface
    {
        return null;
    }

    public function render(): string
    {
        return $this->renderChildren();
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Used by nodes which can have children.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
trait HasChildrenTrait
{
    /**
     * @var NodeInterface[]
     */
    private $children = [];

    public function addChild(NodeInterface $child)
    {
        $this->children[] = $child;
    }

    protected function renderChildren(): string
    {
        $rendered = '';
        foreach ($this->children as $child) {
            $rendered .= $child->render();
        }

        return $rendered;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Used by nodes which can't have children.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
trait IsChildlessTrait
{
    public function addChild(NodeInterface $child)
    {
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Represents a node of the sanitized tree.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface NodeInterface
{
    /**
     * Return this node's parent node if it has one.
     *
     * @return NodeInterface|null
     */
    public function getParent(): ?NodeInterface;

    /**
     * Add a child to this node.
     *
     * @param NodeInterface $node
     */
    public function addChild(NodeInterface $node);

    /**
     * Render this node as a string.
     *
     * @return string
     */
    public function render(): string;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Special node to ignore scripts and all their content.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class ScriptNode extends AbstractTagNode
{
    use IsChildlessTrait;

    public function getTagName(): string
    {
        return 'script';
    }

    public function render(): string
    {
        return '';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Special node to ignore styles and all their content.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class StyleNode extends AbstractTagNode
{
    use IsChildlessTrait;

    public function getTagName(): string
    {
        return 'style';
    }

    public function render(): string
    {
        return '';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

/**
 * Represents a tag node, which has attributes.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface TagNodeInterface extends NodeInterface
{
    /**
     * Return the value of this node given attribute.
     * Return null if the attribute does not exist.
     *
     * @param string $name
     *
     * @return null|string
     */
    public function getAttribute(string $name): ?string;

    /**
     * Set the value of this node given attribute.
     *
     * @param string $name
     * @param string $value
     *
     * @return void
     */
    public function setAttribute(string $name, string $value);
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Node;

use HtmlSanitizer\Sanitizer\StringSanitizerTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class TextNode extends AbstractNode
{
    use IsChildlessTrait;
    use StringSanitizerTrait;

    private $text = '';

    public function __construct(NodeInterface $parent, string $text)
    {
        parent::__construct($parent);

        $this->text = $text;
    }

    public function render(): string
    {
        return $this->encodeHtmlEntities($this->text);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Parser;

use HtmlSanitizer\Exception\ParsingFailedException;
use Masterminds\HTML5;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class MastermindsParser implements ParserInterface
{
    public function parse(string $html): \DOMNode
    {
        try {
            return (new HTML5())->loadHTMLFragment($html);
        } catch (\Throwable $t) {
            throw new ParsingFailedException($this, $t);
        }
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Parser;

use HtmlSanitizer\Exception\ParsingFailedException;

/**
 * A parser transforms a HTML string into a tree of DOMNode objects.
 */
interface ParserInterface
{
    /**
     * Parse a given string and returns a DOMNode tree.
     * This method must throw a ParsingFailedException if parsing failed in order for
     * the sanitizer to catch it and return an empty string.
     *
     * @param string $html
     *
     * @return \DOMNode
     *
     * @throws ParsingFailedException When the parsing fails.
     */
    public function parse(string $html): \DOMNode;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Sanitizer;

/**
 * @internal
 */
trait StringSanitizerTrait
{
    public function encodeHtmlEntities(string $string): string
    {
        $string = htmlspecialchars($string, ENT_QUOTES | ENT_SUBSTITUTE, 'UTF-8');

        // "&#34;" is shorter than "&quot;"
        $string = str_replace('&quot;', '&#34;', $string);

        // Fix several potential issues in how browsers intepret attributes values
        foreach (['+', '=', '@', '`'] as $char) {
            $string = str_replace($char, '&#'.ord($char).';', $string);
        }

        return $string;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Sanitizer;

use HtmlSanitizer\UrlParser\UrlParser;
use function League\Uri\build;

/**
 * @internal
 */
trait UrlSanitizerTrait
{
    /**
     * @var UrlParser|null
     */
    private $parser;

    private function sanitizeUrl(?string $input, array $allowedSchemes, ?array $allowedHosts, bool $forceHttps = false): ?string
    {
        if (!$input) {
            return null;
        }

        if (!$this->parser) {
            $this->parser = new UrlParser();
        }

        $url = $this->parser->parse($input);

        // Malformed URL
        if (!\is_array($url) || !$url) {
            return null;
        }

        // Invalid scheme
        if (!\in_array($url['scheme'], $allowedSchemes, true)) {
            return null;
        }

        // Invalid host
        if ($allowedHosts !== null && !$this->isAllowedHost($url['host'], $allowedHosts)) {
            return null;
        }

        // Force HTTPS
        if ($forceHttps && $url['scheme'] === 'http') {
            $url['scheme'] = 'https';
        }

        return build($url);
    }

    private function isAllowedHost(?string $host, array $allowedHosts): bool
    {
        if ($host === null && \in_array(null, $allowedHosts, true)) {
            return true;
        }

        $parts = array_reverse(explode('.', $host));

        foreach ($allowedHosts as $allowedHost) {
            if ($this->matchAllowedHostParts($parts, array_reverse(explode('.', $allowedHost)))) {
                return true;
            }
        }

        return false;
    }

    private function matchAllowedHostParts(array $uriParts, array $trustedParts): bool
    {
        // Check each chunk of the domain is valid
        foreach ($trustedParts as $key => $trustedPart) {
            if ($uriParts[$key] !== $trustedPart) {
                return false;
            }
        }

        return true;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer;

use HtmlSanitizer\Extension\Basic\BasicExtension;
use HtmlSanitizer\Extension\Code\CodeExtension;
use HtmlSanitizer\Extension\Extra\ExtraExtension;
use HtmlSanitizer\Extension\Iframe\IframeExtension;
use HtmlSanitizer\Extension\Image\ImageExtension;
use HtmlSanitizer\Extension\Listing\ListExtension;
use HtmlSanitizer\Extension\Table\TableExtension;
use HtmlSanitizer\Parser\MastermindsParser;
use HtmlSanitizer\Parser\ParserInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class Sanitizer implements SanitizerInterface
{
    /**
     * @var DomVisitorInterface
     */
    private $domVisitor;

    /**
     * @var int
     */
    private $maxInputLength;

    /**
     * @var ParserInterface
     */
    private $parser;

    public function __construct(DomVisitorInterface $domVisitor, int $maxInputLength, ParserInterface $parser = null)
    {
        $this->domVisitor = $domVisitor;
        $this->maxInputLength = $maxInputLength;
        $this->parser = $parser ?: new MastermindsParser();
    }

    /**
     * Quickly create an already configured sanitizer using the default builder.
     *
     * @param array $config
     *
     * @return SanitizerInterface
     */
    public static function create(array $config): SanitizerInterface
    {
        $builder = new SanitizerBuilder();
        $builder->registerExtension(new BasicExtension());
        $builder->registerExtension(new ListExtension());
        $builder->registerExtension(new ImageExtension());
        $builder->registerExtension(new CodeExtension());
        $builder->registerExtension(new TableExtension());
        $builder->registerExtension(new IframeExtension());
        $builder->registerExtension(new ExtraExtension());

        return $builder->build($config);
    }

    public function sanitize(string $html): string
    {
        // Prevent DOS attack induced by extremely long HTML strings
        if (mb_strlen($html) > $this->maxInputLength) {
            $html = mb_substr($html, 0, $this->maxInputLength);
        }

        /*
         * Only operate on valid UTF-8 strings. This is necessary to prevent cross
         * site scripting issues on Internet Explorer 6. Idea from Drupal (filter_xss).
         */
        if (!$this->isValidUtf8($html)) {
            return '';
        }

        // Remove NULL character
        $html = str_replace(\chr(0), '', $html);

        try {
            $parsed = $this->parser->parse($html);
        } catch (\Exception $exception) {
            return '';
        }

        return $this->domVisitor->visit($parsed)->render();
    }

    /**
     * @param string $html
     *
     * @return bool
     */
    private function isValidUtf8(string $html): bool
    {
        // preg_match() fails silently on strings containing invalid UTF-8.
        return $html === '' || preg_match('/^./us', $html) === 1;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer;

use HtmlSanitizer\Extension\ExtensionInterface;
use HtmlSanitizer\Visitor\ScriptNodeVisitor;
use HtmlSanitizer\Visitor\StyleNodeVisitor;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class SanitizerBuilder implements SanitizerBuilderInterface
{
    /**
     * @var ExtensionInterface[]
     */
    private $extensions = [];

    public function registerExtension(ExtensionInterface $extension)
    {
        $this->extensions[$extension->getName()] = $extension;
    }

    public function build(array $config): SanitizerInterface
    {
        $nodeVisitors = [];

        foreach ($config['extensions'] ?? [] as $extensionName) {
            if (!isset($this->extensions[$extensionName])) {
                throw new \InvalidArgumentException(sprintf(
                    'You have requested a non-existent sanitizer extension "%s" (available extensions: %s)',
                    $extensionName,
                    implode(', ', array_keys($this->extensions))
                ));
            }

            foreach ($this->extensions[$extensionName]->createNodeVisitors($config) as $tagName => $visitor) {
                $nodeVisitors[$tagName] = $visitor;
            }
        }

        // Always required visitors
        $nodeVisitors['script'] = new ScriptNodeVisitor();
        $nodeVisitors['style'] = new StyleNodeVisitor();

        return new Sanitizer(new DomVisitor($nodeVisitors), $config['max_input_length'] ?? 20000);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * Create a sanitizer using sanitizer extensions and configuration.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface SanitizerBuilderInterface
{
    /**
     * Register an extension to use in the sanitizer being built.
     *
     * @param ExtensionInterface $extension
     *
     * @return void
     */
    public function registerExtension(ExtensionInterface $extension);

    /**
     * Build the sanitizer using the given configuration.
     *
     * @param array $config
     *
     * @return SanitizerInterface
     */
    public function build(array $config): SanitizerInterface;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer;

/**
 * Purify a given untrusted HTML source string to return a trustable one.
 * This usually includes removal of all the sources of XSS, and may also includes
 * additional protections like images sources and links targets filters.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface SanitizerInterface
{
    public function sanitize(string $html): string;
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\UrlParser;

use function League\Uri\parse;
use League\Uri\Exception;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class UrlParser
{
    public function parse(string $url): ?array
    {
        if (!$url) {
            return null;
        }

        try {
            $parsed = parse($url);
        } catch (Exception $e) {
            return null;
        }

        return $parsed;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Util;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @final
 */
class Dumper
{
    private static $id;

    public static function dumpDomTree(\DOMNode $tree)
    {
        echo "\ndigraph G {\n";

        self::$id = 0;
        self::dumpDomNode($tree);

        echo "}\n";
    }

    private static function dumpDomNode(\DOMNode $node)
    {
        self::$id++;

        $name = self::$id.'-'.$node->nodeName;
        echo '    "'.$name."\";\n";

        foreach ($node->childNodes ?: [] as $child) {
            $childName = self::dumpDomNode($child);
            echo '    "'.$name.'" -> "'.$childName."\";\n";
        }

        return $name;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
abstract class AbstractNodeVisitor implements NodeVisitorInterface
{
    /**
     * @var array
     */
    protected $config;

    public function __construct(array $config = [])
    {
        $default = $this->getDefaultConfiguration();
        $default['allowed_attributes'] = $this->getDefaultAllowedAttributes();

        $this->config = array_merge($default, $config);
    }

    /**
     * Return this visitor default allowed attributes and their filters.
     *
     * @return array
     */
    public function getDefaultAllowedAttributes(): array
    {
        return [];
    }

    /**
     * Return this visitor additional default configuration.
     * Can only be of depth 1 as it will be merged with the one provided by the user.
     *
     * @return array
     */
    public function getDefaultConfiguration(): array
    {
        return [];
    }

    public function enterNode(\DOMNode $domNode, Cursor $cursor)
    {
    }

    public function leaveNode(\DOMNode $domNode, Cursor $cursor)
    {
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\TagNodeInterface;
use HtmlSanitizer\Node\NodeInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
trait HasChildrenNodeVisitorTrait
{
    use TagVisitorTrait;

    abstract protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface;

    public function enterNode(\DOMNode $domNode, Cursor $cursor)
    {
        $node = $this->createNode($domNode, $cursor);
        if ($node instanceof TagNodeInterface && isset($this->config['allowed_attributes'])) {
            $this->setAttributes($domNode, $node, $this->config['allowed_attributes']);
        }

        $cursor->node->addChild($node);
        $cursor->node = $node;
    }

    public function leaveNode(\DOMNode $domNode, Cursor $cursor)
    {
        $cursor->node = $cursor->node->getParent();
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\TagNodeInterface;
use HtmlSanitizer\Node\NodeInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
trait IsChildlessTagVisitorTrait
{
    use TagVisitorTrait;

    abstract protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface;

    public function enterNode(\DOMNode $domNode, Cursor $cursor)
    {
        $node = $this->createNode($domNode, $cursor);
        if ($node instanceof TagNodeInterface && isset($this->config['allowed_attributes'])) {
            $this->setAttributes($domNode, $node, $this->config['allowed_attributes']);
        }

        $cursor->node->addChild($node);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;

/**
 * A visitor visit supported DOM nodes to decide whether and how to include them in the final output.
 *
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
interface NodeVisitorInterface
{
    /**
     * Whether this visitor supports the DOM node or not in the current context.
     *
     * @param \DOMNode $domNode
     * @param Cursor   $cursor
     *
     * @return bool
     */
    public function supports(\DOMNode $domNode, Cursor $cursor): bool;

    /**
     * Enter the DOM node.
     *
     * @param \DOMNode $domNode
     * @param Cursor   $cursor
     */
    public function enterNode(\DOMNode $domNode, Cursor $cursor);

    /**
     * Leave the DOM node.
     *
     * @param \DOMNode $domNode
     * @param Cursor   $cursor
     */
    public function leaveNode(\DOMNode $domNode, Cursor $cursor);
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\ScriptNode;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class ScriptNodeVisitor extends AbstractNodeVisitor
{
    public function supports(\DOMNode $domNode, Cursor $cursor): bool
    {
        return 'script' === $domNode->nodeName || 'noscript' === $domNode->nodeName;
    }

    public function getDefaultAllowedAttributes(): array
    {
        return [];
    }

    public function enterNode(\DOMNode $domNode, Cursor $cursor)
    {
        $node = new ScriptNode($cursor->node);

        $cursor->node->addChild($node);
        $cursor->node = $node;
    }

    public function leaveNode(\DOMNode $domNode, Cursor $cursor)
    {
        $cursor->node = $cursor->node->getParent();
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\StyleNode;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
class StyleNodeVisitor extends AbstractNodeVisitor
{
    public function supports(\DOMNode $domNode, Cursor $cursor): bool
    {
        return 'style' === $domNode->nodeName;
    }

    public function getDefaultAllowedAttributes(): array
    {
        return [];
    }

    public function enterNode(\DOMNode $domNode, Cursor $cursor)
    {
        $node = new StyleNode($cursor->node);

        $cursor->node->addChild($node);
        $cursor->node = $node;
    }

    public function leaveNode(\DOMNode $domNode, Cursor $cursor)
    {
        $cursor->node = $cursor->node->getParent();
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace HtmlSanitizer\Visitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\TagNodeInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 *
 * @internal
 */
trait TagVisitorTrait
{
    abstract protected function getDomNodeName(): string;

    public function supports(\DOMNode $domNode, Cursor $cursor): bool
    {
        return $this->getDomNodeName() === $domNode->nodeName;
    }

    /**
     * Set attributes from a DOM node to a sanitized node.
     *
     * @param \DOMNode $domNode
     * @param TagNodeInterface $node
     * @param array $allowedAttributes
     */
    private function setAttributes(\DOMNode $domNode, TagNodeInterface $node, array $allowedAttributes = [])
    {
        if (!count($domNode->attributes)) {
            return;
        }

        /** @var \DOMAttr $attribute */
        foreach ($domNode->attributes as $attribute) {
            $name = strtolower($attribute->name);

            if (in_array($name, $allowedAttributes)) {
                $node->setAttribute($name, $attribute->value);
            }
        }
    }

    /**
     * Read the value of a DOMNode attribute.
     *
     * @param \DOMNode $domNode
     * @param string $name
     *
     * @return null|string
     */
    private function getAttribute(\DOMNode $domNode, string $name): ?string
    {
        if (!count($domNode->attributes)) {
            return null;
        }

        /** @var \DOMAttr $attribute */
        foreach ($domNode->attributes as $attribute) {
            if ($attribute->name === $name) {
                return $attribute->value;
            }
        }

        return null;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer;

use HtmlSanitizer\SanitizerBuilder;
use HtmlSanitizer\SanitizerInterface;
use PHPUnit\Framework\TestCase;

abstract class AbstractSanitizerTest extends TestCase
{
    abstract public function createSanitizer(): SanitizerInterface;

    public function provideFixtures(): array
    {
        // Fixtures shared by all sanitizers
        return [

            [
                'hello world',
                'hello world',
            ],
            [
                '&lt;hello world&gt;',
                '&lt;hello world&gt;',
            ],
            [
                '< Hello',
                ' Hello',
            ],
            [
                'Lorem & Ipsum',
                'Lorem &amp; Ipsum',
            ],

            // Unknown tag
            [
                '<unknown>Lorem ipsum</unknown>',
                'Lorem ipsum',
            ],

            // Scripts
            [
                '<script>alert(\'ok\');</script>',
                '',
            ],
            [
                '<noscript>Lorem ipsum</noscript>',
                '',
            ],

            // Styles
            [
                '<style>body { background: red; }</style>',
                '',
            ],

            // Comments
            [
                'Lorem ipsum dolor sit amet, consectetur<!--if[true]> <script>alert(1337)</script> -->',
                'Lorem ipsum dolor sit amet, consectetur',
            ],
            [
                'Lorem ipsum<![CDATA[ <!-- ]]> <script>alert(1337)</script> <!-- -->',
                'Lorem ipsum  ',
            ],

        ];
    }

    public function provideSanitizerInput()
    {
        foreach ($this->provideFixtures() as $fixture) {
            yield $fixture[0] => [$fixture[0], $fixture[1]];
        }
    }

    /**
     * @dataProvider provideSanitizerInput
     */
    public function testSanitize($input, $expectedOutput)
    {
        $this->assertEquals($expectedOutput, $this->createSanitizer()->sanitize($input));
    }

    public function testRemoveNullByte()
    {
        $this->assertSame('Null byte', $this->createSanitizer()->sanitize("Null byte\0"));
        $this->assertSame('Null byte', $this->createSanitizer()->sanitize("Null byte&#0;"));
    }

    public function testDeeplyNestedTagDos()
    {
        $this->assertNotEmpty($this->createSanitizer()->sanitize(str_repeat('<div>T', 10000)));
    }

    /**
     * @expectedException \InvalidArgumentException
     */
    public function testThrowInvalidExtension()
    {
        $builder = new SanitizerBuilder();
        $builder->build(['extensions' => ['invalid']]);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer;

use HtmlSanitizer\Sanitizer;
use HtmlSanitizer\SanitizerInterface;

class EmptySanitizerTest extends AbstractSanitizerTest
{
    public function createSanitizer(): SanitizerInterface
    {
        return Sanitizer::create([]);
    }

    public function provideFixtures(): array
    {
        return array_merge(parent::provideFixtures(), [

            /*
             * Normal tags
             */

            [
                '<abbr class="foo">Lorem ipsum</abbr>',
                'Lorem ipsum',
            ],
            [
                '<a href="https://trusted.com" title="Link title" class="foo">Lorem ipsum</a>',
                'Lorem ipsum',
            ],
            [
                '<blockquote class="foo">Lorem ipsum</blockquote>',
                'Lorem ipsum',
            ],
            [
                'Lorem ipsum <br class="foo">dolor sit amet <br class="foo" />consectetur adipisicing.',
                'Lorem ipsum dolor sit amet consectetur adipisicing.',
            ],
            [
                '<caption class="foo">Lorem ipsum</caption>',
                'Lorem ipsum',
            ],
            [
                '<code class="foo">Lorem ipsum</code>',
                'Lorem ipsum',
            ],
            [
                '<dd class="foo">Lorem ipsum</dd>',
                'Lorem ipsum',
            ],
            [
                '<del class="foo">Lorem ipsum</del>',
                'Lorem ipsum',
            ],
            [
                '<div class="foo">Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
                'Lorem ipsum dolor sit amet, consectetur adipisicing elit.',
            ],
            [
                '<dl class="foo">Lorem ipsum</dl>',
                'Lorem ipsum',
            ],
            [
                '<dt class="foo">Lorem ipsum</dt>',
                'Lorem ipsum',
            ],
            [
                '<em class="foo">Lorem ipsum</em>',
                'Lorem ipsum',
            ],
            [
                '<figcaption class="foo">Lorem ipsum</figcaption>',
                'Lorem ipsum',
            ],
            [
                '<figure class="foo">Lorem ipsum</figure>',
                'Lorem ipsum',
            ],
            [
                '<h1 class="foo">Lorem ipsum</h1>',
                'Lorem ipsum',
            ],
            [
                '<h2 class="foo">Lorem ipsum</h2>',
                'Lorem ipsum',
            ],
            [
                '<h3 class="foo">Lorem ipsum</h3>',
                'Lorem ipsum',
            ],
            [
                '<h4 class="foo">Lorem ipsum</h4>',
                'Lorem ipsum',
            ],
            [
                '<h5 class="foo">Lorem ipsum</h5>',
                'Lorem ipsum',
            ],
            [
                '<h6 class="foo">Lorem ipsum</h6>',
                'Lorem ipsum',
            ],
            [
                '<hr class="foo" />',
                '',
            ],
            [
                '<iframe src="/frame/example" width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
                'Lorem ipsum',
            ],
            [
                '<iframe>Lorem ipsum</iframe>',
                'Lorem ipsum',
            ],
            [
                '<img src="/img/example.jpg" alt="Image alternative text" title="Image title" class="foo" />',
                '',
            ],
            [
                '<img src="http://trusted.com/img/example.jpg" alt="Image alternative text" title="Image title" class="foo" />',
                '',
            ],
            [
                '<img />',
                '',
            ],
            [
                '<i class="foo">Lorem ipsum</i>',
                'Lorem ipsum',
            ],
            [
                '<li class="foo">Lorem ipsum</li>',
                'Lorem ipsum',
            ],
            [
                '<ol class="foo">Lorem ipsum</ol>',
                'Lorem ipsum',
            ],
            [
                '<p class="foo">Lorem ipsum</p>',
                'Lorem ipsum',
            ],
            [
                '<pre class="foo">Lorem ipsum</pre>',
                'Lorem ipsum',
            ],
            [
                '<q class="foo">Lorem ipsum</q>',
                'Lorem ipsum',
            ],
            [
                '<rp class="foo">Lorem ipsum</rp>',
                'Lorem ipsum',
            ],
            [
                '<rt class="foo">Lorem ipsum</rt>',
                'Lorem ipsum',
            ],
            [
                '<ruby class="foo">Lorem ipsum</ruby>',
                'Lorem ipsum',
            ],
            [
                '<small class="foo">Lorem ipsum</small>',
                'Lorem ipsum',
            ],
            [
                '<span class="foo">Lorem ipsum</span>',
                'Lorem ipsum',
            ],
            [
                '<strong class="foo">Lorem ipsum</strong>',
                'Lorem ipsum',
            ],
            [
                '<b class="foo">Lorem ipsum</b>',
                'Lorem ipsum',
            ],
            [
                '<sub class="foo">Lorem ipsum</sub>',
                'Lorem ipsum',
            ],
            [
                '<sup class="foo">Lorem ipsum</sup>',
                'Lorem ipsum',
            ],
            [
                '<table class="foo">Lorem ipsum</table>',
                'Lorem ipsum',
            ],
            [
                '<tbody class="foo">Lorem ipsum</tbody>',
                'Lorem ipsum',
            ],
            [
                '<td class="foo">Lorem ipsum</td>',
                'Lorem ipsum',
            ],
            [
                '<tfoot class="foo">Lorem ipsum</tfoot>',
                'Lorem ipsum',
            ],

            /*
             * Scripts
             */

            [
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.<script>alert(\'ok\');</script></div>',
                'Lorem ipsum dolor sit amet, consectetur adipisicing elit.',
            ],
            [
                '<figure><img src="/img/example.jpg" onclick="alert(\'ok\')" /></figure>',
                '',
            ],
            [
                '<a href="javascript:alert(\'ok\')">Lorem ipsum dolor sit amet, consectetur adipisicing elit.</a>',
                'Lorem ipsum dolor sit amet, consectetur adipisicing elit.',
            ],

            /*
             * Styles
             */

            [
                '<div>Lorem ipsum dolor sit amet, consectetur.<style>body { background: red; }</style></div>',
                'Lorem ipsum dolor sit amet, consectetur.',
            ],
            [
                '<img src="/img/example.jpg" style="position:absolute;top:0;left:0;width:9000px;height:9000px;" />',
                '',
            ],
            [
                '<a style="font-size: 40px; color: red;">Lorem ipsum dolor sit amet, consectetur.</a>',
                'Lorem ipsum dolor sit amet, consectetur.',
            ],

        ]);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Extension;

use HtmlSanitizer\Extension\ExtensionInterface;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
class CustomExtension implements ExtensionInterface
{
    public function getName(): string
    {
        return 'custom';
    }

    public function createNodeVisitors(array $config = []): array
    {
        return [
            'custom' => new NodeVisitor\CustomNodeVisitor($config['tags']['custom'] ?? []),
        ];
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Extension\Node;

use HtmlSanitizer\Node\AbstractTagNode;
use HtmlSanitizer\Node\HasChildrenTrait;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
class CustomNode extends AbstractTagNode
{
    use HasChildrenTrait;

    public function getTagName(): string
    {
        return 'custom';
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Extension\NodeVisitor;

use HtmlSanitizer\Model\Cursor;
use HtmlSanitizer\Node\NodeInterface;
use HtmlSanitizer\Visitor\AbstractNodeVisitor;
use HtmlSanitizer\Visitor\HasChildrenNodeVisitorTrait;
use Tests\HtmlSanitizer\Extension\Node\CustomNode;

/**
 * @author Titouan Galopin <galopintitouan@gmail.com>
 */
class CustomNodeVisitor extends AbstractNodeVisitor
{
    use HasChildrenNodeVisitorTrait;

    protected function getDomNodeName(): string
    {
        return 'custom';
    }

    public function getDefaultAllowedAttributes(): array
    {
        return [
            'class', 'width', 'height'
        ];
    }

    public function getDefaultConfiguration(): array
    {
        return [
            'custom_data' => null,
        ];
    }

    protected function createNode(\DOMNode $domNode, Cursor $cursor): NodeInterface
    {
        $node = new CustomNode($cursor->node);
        $node->setAttribute('data-custom', $this->config['custom_data']);

        return $node;
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer;

use HtmlSanitizer\SanitizerBuilder;
use HtmlSanitizer\SanitizerInterface;
use Tests\HtmlSanitizer\Extension\CustomExtension;

class ExtensionSanitizerTest extends EmptySanitizerTest
{
    public function createSanitizer(): SanitizerInterface
    {
        $builder = new SanitizerBuilder();
        $builder->registerExtension(new CustomExtension());

        return $builder->build([
            'extensions' => ['custom'],
            'tags' => [
                'custom' => [
                    'custom_data' => 'foo',
                ],
            ],
        ]);
    }

    public function provideFixtures(): array
    {
        return array_merge(parent::provideFixtures(), [
            [
                '<custom>Lorem ipsum</custom>',
                '<custom data-custom="foo">Lorem ipsum</custom>',
            ],
        ]);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer;

use HtmlSanitizer\Sanitizer;
use HtmlSanitizer\SanitizerInterface;

class FullSanitizerTest extends AbstractSanitizerTest
{
    public function createSanitizer(): SanitizerInterface
    {
        return Sanitizer::create([
            'extensions' => ['basic', 'code', 'image', 'list', 'table', 'iframe', 'extra'],
            'tags' => [
                'a' => [
                    'allowed_hosts' => ['trusted.com', 'external.com'],
                ],
                'img' => [
                    'allowed_hosts' => ['trusted.com'],
                    'force_https' => true,
                ],
                'iframe' => [
                    'allowed_hosts' => ['trusted.com'],
                    'force_https' => true,
                ],
            ],
        ]);
    }

    public function provideFixtures(): array
    {
        return array_merge(parent::provideFixtures(), [

            /*
             * Normal tags
             */

            [
                '<abbr class="foo">Lorem ipsum</abbr>',
                '<abbr>Lorem ipsum</abbr>',
            ],
            [
                '<a href="https://trusted.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="https://trusted.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<a href="https://untrusted.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a title="Link title">Lorem ipsum</a>',
            ],
            [
                '<a href="https://external.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="https://external.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<a href="mailto:test&#64;gmail.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="mailto:test&#64;gmail.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<blockquote class="foo">Lorem ipsum</blockquote>',
                '<blockquote>Lorem ipsum</blockquote>',
            ],
            [
                'Lorem ipsum <br class="foo">dolor sit amet <br class="foo" />consectetur adipisicing.',
                'Lorem ipsum <br />dolor sit amet <br />consectetur adipisicing.',
            ],
            [
                '<caption class="foo">Lorem ipsum</caption>',
                '<caption>Lorem ipsum</caption>',
            ],
            [
                '<code class="foo">Lorem ipsum</code>',
                '<code>Lorem ipsum</code>',
            ],
            [
                '<dd class="foo">Lorem ipsum</dd>',
                '<dd>Lorem ipsum</dd>',
            ],
            [
                '<del class="foo">Lorem ipsum</del>',
                '<del>Lorem ipsum</del>',
            ],
            [
                '<div class="foo">Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
            ],
            [
                '<dl class="foo">Lorem ipsum</dl>',
                '<dl>Lorem ipsum</dl>',
            ],
            [
                '<dt class="foo">Lorem ipsum</dt>',
                '<dt>Lorem ipsum</dt>',
            ],
            [
                '<em class="foo">Lorem ipsum</em>',
                '<em>Lorem ipsum</em>',
            ],
            [
                '<figcaption class="foo">Lorem ipsum</figcaption>',
                '<figcaption>Lorem ipsum</figcaption>',
            ],
            [
                '<figure class="foo">Lorem ipsum</figure>',
                '<figure>Lorem ipsum</figure>',
            ],
            [
                '<h1 class="foo">Lorem ipsum</h1>',
                '<h1>Lorem ipsum</h1>',
            ],
            [
                '<h2 class="foo">Lorem ipsum</h2>',
                '<h2>Lorem ipsum</h2>',
            ],
            [
                '<h3 class="foo">Lorem ipsum</h3>',
                '<h3>Lorem ipsum</h3>',
            ],
            [
                '<h4 class="foo">Lorem ipsum</h4>',
                '<h4>Lorem ipsum</h4>',
            ],
            [
                '<h5 class="foo">Lorem ipsum</h5>',
                '<h5>Lorem ipsum</h5>',
            ],
            [
                '<h6 class="foo">Lorem ipsum</h6>',
                '<h6>Lorem ipsum</h6>',
            ],
            [
                '<hr class="foo" />',
                '<hr />',
            ],
            [
                '<iframe src="/frame/example" width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
                '<iframe width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
            ],
            [
                '<iframe src="http://trusted.com/frame/example" width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
                '<iframe src="https://trusted.com/frame/example" width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
            ],
            [
                '<iframe src="http://untrusted.com/frame/example" width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
                '<iframe width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
            ],
            [
                '<iframe>Lorem ipsum</iframe>',
                '<iframe>Lorem ipsum</iframe>',
            ],
            [
                '<img src="/img/example.jpg" alt="Image alternative text" title="Image title" class="foo">',
                '<img alt="Image alternative text" title="Image title" />',
            ],
            [
                '<img src="http://trusted.com/img/example.jpg" alt="Image alternative text" title="Image title" class="foo" />',
                '<img src="https://trusted.com/img/example.jpg" alt="Image alternative text" title="Image title" />',
            ],
            [
                '<img src="http://untrusted.com/img/example.jpg" alt="Image alternative text" title="Image title" class="foo" />',
                '<img alt="Image alternative text" title="Image title" />',
            ],
            [
                '<img />',
                '<img />',
            ],
            [
                '<img title="" />',
                '<img title />',
            ],
            [
                '<i class="foo">Lorem ipsum</i>',
                '<i>Lorem ipsum</i>',
            ],
            [
                '<li class="foo">Lorem ipsum</li>',
                '<li>Lorem ipsum</li>',
            ],
            [
                '<ol class="foo">Lorem ipsum</ol>',
                '<ol>Lorem ipsum</ol>',
            ],
            [
                '<p class="foo">Lorem ipsum</p>',
                '<p>Lorem ipsum</p>',
            ],
            [
                '<pre class="foo">Lorem ipsum</pre>',
                '<pre>Lorem ipsum</pre>',
            ],
            [
                '<q class="foo">Lorem ipsum</q>',
                '<q>Lorem ipsum</q>',
            ],
            [
                '<rp class="foo">Lorem ipsum</rp>',
                '<rp>Lorem ipsum</rp>',
            ],
            [
                '<rt class="foo">Lorem ipsum</rt>',
                '<rt>Lorem ipsum</rt>',
            ],
            [
                '<ruby class="foo">Lorem ipsum</ruby>',
                '<ruby>Lorem ipsum</ruby>',
            ],
            [
                '<small class="foo">Lorem ipsum</small>',
                '<small>Lorem ipsum</small>',
            ],
            [
                '<span class="foo">Lorem ipsum</span>',
                '<span>Lorem ipsum</span>',
            ],
            [
                '<strong class="foo">Lorem ipsum</strong>',
                '<strong>Lorem ipsum</strong>',
            ],
            [
                '<b class="foo">Lorem ipsum</b>',
                '<strong>Lorem ipsum</strong>',
            ],
            [
                '<sub class="foo">Lorem ipsum</sub>',
                '<sub>Lorem ipsum</sub>',
            ],
            [
                '<sup class="foo">Lorem ipsum</sup>',
                '<sup>Lorem ipsum</sup>',
            ],
            [
                '<table class="foo">Lorem ipsum</table>',
                '<table>Lorem ipsum</table>',
            ],
            [
                '<tbody class="foo">Lorem ipsum</tbody>',
                '<tbody>Lorem ipsum</tbody>',
            ],
            [
                '<td class="foo">Lorem ipsum</td>',
                '<td>Lorem ipsum</td>',
            ],
            [
                '<tfoot class="foo">Lorem ipsum</tfoot>',
                '<tfoot>Lorem ipsum</tfoot>',
            ],
            [
                '<thead class="foo">Lorem ipsum</thead>',
                '<thead>Lorem ipsum</thead>',
            ],
            [
                '<th class="foo">Lorem ipsum</th>',
                '<th>Lorem ipsum</th>',
            ],
            [
                '<tr class="foo">Lorem ipsum</tr>',
                '<tr>Lorem ipsum</tr>',
            ],
            [
                '<ul class="foo">Lorem ipsum</ul>',
                '<ul>Lorem ipsum</ul>',
            ],

            /*
             * Links
             */

            [
                '<a href="mailto:test&#64;gmail.com">Test</a>',
                '<a href="mailto:test&#64;gmail.com">Test</a>',
            ],
            [
                '<a href="mailto:alert(\'ok\')">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="javascript:alert(\'ok\')">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="javascript://%0Aalert(document.cookie)">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="http://untrusted.com" onclick="alert(\'ok\')">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="https://trusted.com">Test</a>',
                '<a href="https://trusted.com">Test</a>',
            ],
            [
                '<a>Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],
            [
                '<a href="&#106;&#97;&#118;&#97;&#115;&#99;&#114;&#105;&#112;&#116;&#58;&#97;&#108;&#101;&#114;&#116;&#40;&#39;&#88;&#83;&#83;&#39;&#41;">Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],
            [
                '<a href="http://trusted.com/index.html#this:stuff">Lorem ipsum</a>',
                '<a href="http://trusted.com/index.html#this:stuff">Lorem ipsum</a>',
            ],
            [
                '<a href="java\0&#14;\t\r\n script:alert(\\\'foo\\\')">Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],
            [
                '<a href= onmouseover="alert(\\\'XSS\\\');">Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],

            /*
             * Scripts
             */

            [
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.<script>alert(\'ok\');</script></div>',
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
            ],
            [
                '<figure><img src="https://trusted.com/img/example.jpg" onclick="alert(\'ok\')" /></figure>',
                '<figure><img src="https://trusted.com/img/example.jpg" /></figure>',
            ],
            [
                '<a href="javascript:alert(\'ok\')">Lorem ipsum dolor sit amet, consectetur adipisicing elit.</a>',
                '<a>Lorem ipsum dolor sit amet, consectetur adipisicing elit.</a>',
            ],
            [
                '<img src= onmouseover="alert(\'XSS\');" />',
                '<img />',
            ],
            [
                '<<img src="javascript:evil"/>img src="javascript:evil"/>',
                '<img />img src&#61;&#34;javascript:evil&#34;/&gt;',
            ],
            [
                '<<a href="javascript:evil"/>a href="javascript:evil"/>',
                '<a>a href&#61;&#34;javascript:evil&#34;/&gt;</a>',
            ],
            [
                '!<textarea>&lt;/textarea&gt;&lt;svg/onload=prompt`xs`&gt;</textarea>!',
                '!&lt;/textarea&gt;&lt;svg/onload&#61;prompt&#96;xs&#96;&gt;!',
            ],
            [
                '<iframe src= onmouseover="alert(\'XSS\');" />',
                '<iframe></iframe>',
            ],
            [
                '<<iframe src="javascript:evil"/>iframe src="javascript:evil"/>',
                '<iframe>iframe src&#61;&#34;javascript:evil&#34;/&gt;</iframe>',
            ],
            [
                '<scr<script>ipt>alert(1)</script>',
                '',
            ],
            [
                '<scr<a>ipt>alert(1)</script>',
                '<a>ipt&gt;alert(1)</a>',
            ],

            /*
             * Styles
             */

            [
                '<div>Lorem ipsum dolor sit amet, consectetur.<style>body { background: red; }</style></div>',
                '<div>Lorem ipsum dolor sit amet, consectetur.</div>',
            ],
            [
                '<img src="https://trusted.com/img/example.jpg" style="position:absolute;top:0;left:0;width:9000px;height:9000px;" />',
                '<img src="https://trusted.com/img/example.jpg" />',
            ],
            [
                '<a style="font-size: 40px; color: red;">Lorem ipsum dolor sit amet, consectetur.</a>',
                '<a>Lorem ipsum dolor sit amet, consectetur.</a>',
            ],

        ]);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer;

use HtmlSanitizer\Sanitizer;
use HtmlSanitizer\SanitizerInterface;
use PHPUnit\Framework\TestCase;

class MalformedHtmlTest extends TestCase
{
    public function testSanitizeMalformedUrl()
    {
        // Use rtrim to remove end line should be able to change without consequence on the validity of the test
        $input = rtrim(file_get_contents(__DIR__.'/Fixtures/malformed/input.html'));
        $expectedOutput = rtrim(file_get_contents(__DIR__.'/Fixtures/malformed/output.html'));

        $this->assertEquals($expectedOutput, $this->createSanitizer()->sanitize($input));
    }

    private function createSanitizer(): SanitizerInterface
    {
        return Sanitizer::create(['extensions' => ['basic', 'code', 'image', 'list', 'table', 'iframe', 'extra']]);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Sanitizer;

use HtmlSanitizer\Extension\Basic\Sanitizer\AHrefSanitizer;
use PHPUnit\Framework\TestCase;

class AHrefSanitizerTest extends TestCase
{
    public function provideUrls()
    {
        // Simple cases
        yield [
            'allowedHosts' => null,
            'allowMailTo' => false,
            'forceHttps' => false,
            'input' => 'https://trusted.com/link.php',
            'output' => 'https://trusted.com/link.php',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowMailTo' => false,
            'forceHttps' => false,
            'input' => 'https://trusted.com/link.php',
            'output' => 'https://trusted.com/link.php',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowMailTo' => false,
            'forceHttps' => false,
            'input' => 'https://untrusted.com/link.php',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowMailTo' => false,
            'forceHttps' => false,
            'input' => '/link.php',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowMailTo' => true,
            'forceHttps' => false,
            'input' => '/link.php',
            'output' => null,
        ];

        // Force HTTPS
        yield [
            'allowedHosts' => ['trusted.com'],
            'allowMailTo' => false,
            'forceHttps' => true,
            'input' => 'http://trusted.com/link.php',
            'output' => 'https://trusted.com/link.php',
        ];

        // Data-URI not allowed
        yield [
            'allowedHosts' => null,
            'allowMailTo' => true,
            'forceHttps' => false,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowMailTo' => true,
            'forceHttps' => true,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];

        // MailTo
        yield [
            'allowedHosts' => null,
            'allowMailTo' => false,
            'forceHttps' => false,
            'input' => 'mailto:test@gmail.com',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowMailTo' => true,
            'forceHttps' => false,
            'input' => 'mailto:test@gmail.com',
            'output' => 'mailto:test@gmail.com',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowMailTo' => true,
            'forceHttps' => false,
            'input' => 'mailto:test@gmail.com',
            'output' => 'mailto:test@gmail.com',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowMailTo' => true,
            'forceHttps' => true,
            'input' => 'mailto:test@gmail.com',
            'output' => 'mailto:test@gmail.com',
        ];

        yield [
            'allowedHosts' => null,
            'allowMailTo' => true,
            'forceHttps' => false,
            'input' => 'mailto:invalid',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowMailTo' => true,
            'forceHttps' => false,
            'input' => 'mailto:',
            'output' => null,
        ];
    }

    /**
     * @dataProvider provideUrls
     */
    public function testSanitize($allowedHosts, $allowMailTo, $forceHttps, $input, $expected)
    {
        $this->assertSame($expected, (new AHrefSanitizer($allowedHosts, $allowMailTo, $forceHttps))->sanitize($input));
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Sanitizer;

use HtmlSanitizer\Extension\Iframe\Sanitizer\IframeSrcSanitizer;
use PHPUnit\Framework\TestCase;

class IframeSrcSanitizerTest extends TestCase
{
    public function provideUrls()
    {
        // Simple cases
        yield [
            'allowedHosts' => null,
            'forceHttps' => false,
            'input' => 'https://trusted.com/iframe.php',
            'output' => 'https://trusted.com/iframe.php',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'input' => 'https://trusted.com/iframe.php',
            'output' => 'https://trusted.com/iframe.php',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'input' => 'https://untrusted.com/iframe.php',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'forceHttps' => false,
            'input' => '/iframe.php',
            'output' => null,
        ];

        // Force HTTPS
        yield [
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => true,
            'input' => 'http://trusted.com/iframe.php',
            'output' => 'https://trusted.com/iframe.php',
        ];

        // Data-URI not allowed
        yield [
            'allowedHosts' => null,
            'forceHttps' => false,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'forceHttps' => true,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];
    }

    /**
     * @dataProvider provideUrls
     */
    public function testSanitize($allowedHosts, $forceHttps, $input, $expected)
    {
        $this->assertSame($expected, (new IframeSrcSanitizer($allowedHosts, $forceHttps))->sanitize($input));
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Sanitizer;

use HtmlSanitizer\Extension\Image\Sanitizer\ImgSrcSanitizer;
use PHPUnit\Framework\TestCase;

class ImgSrcSanitizerTest extends TestCase
{
    public function provideUrls()
    {
        // Simple cases
        yield [
            'allowedHosts' => null,
            'allowDataUri' => false,
            'forceHttps' => false,
            'input' => 'https://trusted.com/image.php',
            'output' => 'https://trusted.com/image.php',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowDataUri' => false,
            'forceHttps' => false,
            'input' => 'https://trusted.com/image.php',
            'output' => 'https://trusted.com/image.php',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowDataUri' => false,
            'forceHttps' => false,
            'input' => 'https://untrusted.com/image.php',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowDataUri' => false,
            'forceHttps' => false,
            'input' => '/image.php',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowDataUri' => true,
            'forceHttps' => false,
            'input' => '/image.php',
            'output' => null,
        ];

        // Force HTTPS
        yield [
            'allowedHosts' => ['trusted.com'],
            'allowDataUri' => false,
            'forceHttps' => true,
            'input' => 'http://trusted.com/image.php',
            'output' => 'https://trusted.com/image.php',
        ];

        // Data-URI
        yield [
            'allowedHosts' => null,
            'allowDataUri' => false,
            'forceHttps' => false,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowDataUri' => true,
            'forceHttps' => false,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
        ];

        yield [
            'allowedHosts' => ['trusted.com'],
            'allowDataUri' => true,
            'forceHttps' => false,
            'input' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => 'data:image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
        ];

        yield [
            'allowedHosts' => null,
            'allowDataUri' => true,
            'forceHttps' => false,
            'input' => 'data://image/gif;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowDataUri' => true,
            'forceHttps' => false,
            'input' => 'data:',
            'output' => null,
        ];

        yield [
            'allowedHosts' => null,
            'allowDataUri' => true,
            'forceHttps' => false,
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'output' => null,
        ];
    }

    /**
     * @dataProvider provideUrls
     */
    public function testSanitize($allowedHosts, $allowDataUri, $forceHttps, $input, $expected)
    {
        $this->assertSame($expected, (new ImgSrcSanitizer($allowedHosts, $allowDataUri, $forceHttps))->sanitize($input));
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Sanitizer;

use HtmlSanitizer\Sanitizer\StringSanitizerTrait;
use PHPUnit\Framework\TestCase;

class StringSanitizerTraitTest extends TestCase
{
    use StringSanitizerTrait;

    public function provideEncodeHtmlEntites()
    {
        $entities = [
            '' => '',
            '"' => '&#34;',
            '\'' => '&#039;',
            '&' => '&amp;',
            "<" => "&lt;",
            ">" => "&gt;",
            "&lt;" => "&amp;lt;",
            "&gt;" => "&amp;gt;",
            '+' => '&#43;',
            '=' => '&#61;',
            '@' => '&#64;',
            '`' => '&#96;',
        ];

        foreach ($entities as $input => $expected) {
            yield $input => [$input, $expected];
        }
    }

    /**
     * @dataProvider provideEncodeHtmlEntites
     */
    public function testEncodeHtmlEntites($input, $expected)
    {
        $this->assertEquals($expected, $this->encodeHtmlEntities($input));
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\Sanitizer;

use HtmlSanitizer\Sanitizer\UrlSanitizerTrait;
use PHPUnit\Framework\TestCase;

class UrlSanitizerTraitTest extends TestCase
{
    use UrlSanitizerTrait;

    public function provideSanitizeUrls()
    {
        // Simple accepted cases
        yield [
            'input' => 'https://trusted.com/link.php',
            'allowedSchemes' => ['https'],
            'allowedHosts' => null,
            'forceHttps' => false,
            'output' => 'https://trusted.com/link.php',
        ];

        yield [
            'input' => 'https://trusted.com/link.php',
            'allowedSchemes' => ['https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => 'https://trusted.com/link.php',
        ];

        yield [
            'input' => 'http://trusted.com/link.php',
            'allowedSchemes' => ['http'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => 'http://trusted.com/link.php',
        ];

        yield [
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'allowedSchemes' => ['data'],
            'allowedHosts' => null,
            'forceHttps' => false,
            'output' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
        ];

        // Simple filtered cases
        yield [
            'input' => 'ws://trusted.com/link.php',
            'allowedSchemes' => ['http'],
            'allowedHosts' => null,
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'ws://trusted.com/link.php',
            'allowedSchemes' => ['http'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'https://trusted.com/link.php',
            'allowedSchemes' => ['http'],
            'allowedHosts' => null,
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'https://untrusted.com/link.php',
            'allowedSchemes' => ['https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'http://untrusted.com/link.php',
            'allowedSchemes' => ['http'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'allowedSchemes' => ['http'],
            'allowedHosts' => null,
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'allowedSchemes' => ['http'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => null,
        ];

        // Allow null host (data scheme for instance)
        yield [
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'allowedSchemes' => ['http', 'https', 'data'],
            'allowedHosts' => ['trusted.com', null],
            'forceHttps' => false,
            'output' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
        ];

        // Force HTTPS
        yield [
            'input' => 'http://trusted.com/link.php',
            'allowedSchemes' => ['http', 'https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => true,
            'output' => 'https://trusted.com/link.php',
        ];

        yield [
            'input' => 'https://trusted.com/link.php',
            'allowedSchemes' => ['http', 'https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => true,
            'output' => 'https://trusted.com/link.php',
        ];

        yield [
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'allowedSchemes' => ['http', 'https', 'data'],
            'allowedHosts' => null,
            'forceHttps' => true,
            'output' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
        ];

        yield [
            'input' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
            'allowedSchemes' => ['http', 'https', 'data'],
            'allowedHosts' => ['trusted.com', null],
            'forceHttps' => true,
            'output' => 'data:text/plain;base64,R0lGODlhAQABAIAAAAAAAP///yH5BAEAAAAALAAAAAABAAEAAAIBRAA7',
        ];

        // Domain matching
        yield [
            'input' => 'https://subdomain.trusted.com/link.php',
            'allowedSchemes' => ['http', 'https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => 'https://subdomain.trusted.com/link.php',
        ];

        yield [
            'input' => 'https://subdomain.trusted.com.untrusted.com/link.php',
            'allowedSchemes' => ['http', 'https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => null,
        ];

        yield [
            'input' => 'https://deep.subdomain.trusted.com/link.php',
            'allowedSchemes' => ['http', 'https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => 'https://deep.subdomain.trusted.com/link.php',
        ];

        yield [
            'input' => 'https://deep.subdomain.trusted.com.untrusted.com/link.php',
            'allowedSchemes' => ['http', 'https'],
            'allowedHosts' => ['trusted.com'],
            'forceHttps' => false,
            'output' => null,
        ];
    }

    /**
     * @dataProvider provideSanitizeUrls
     */
    public function testSanitizeUrl($input, $allowedSchemes, $allowedHosts, $forceHttps, $expected)
    {
        $this->assertEquals($expected, $this->sanitizeUrl($input, $allowedSchemes, $allowedHosts, $forceHttps));
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer;

use HtmlSanitizer\Sanitizer;
use HtmlSanitizer\SanitizerInterface;

class SimpleSanitizerTest extends AbstractSanitizerTest
{
    public function createSanitizer(): SanitizerInterface
    {
        return Sanitizer::create(['extensions' => ['basic']]);
    }

    public function provideFixtures(): array
    {
        return array_merge(parent::provideFixtures(), [

            /*
             * Normal tags
             */

            [
                '<abbr class="foo">Lorem ipsum</abbr>',
                'Lorem ipsum',
            ],
            [
                '<a href="https://trusted.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="https://trusted.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<a href="https://untrusted.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="https://untrusted.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<a href="https://external.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="https://external.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<a href="mailto:test&#64;gmail.com" title="Link title" class="foo">Lorem ipsum</a>',
                '<a href="mailto:test&#64;gmail.com" title="Link title">Lorem ipsum</a>',
            ],
            [
                '<blockquote class="foo">Lorem ipsum</blockquote>',
                '<blockquote>Lorem ipsum</blockquote>',
            ],
            [
                'Lorem ipsum <br class="foo">dolor sit amet <br class="foo" />consectetur adipisicing.',
                'Lorem ipsum <br />dolor sit amet <br />consectetur adipisicing.',
            ],
            [
                '<caption class="foo">Lorem ipsum</caption>',
                'Lorem ipsum',
            ],
            [
                '<code class="foo">Lorem ipsum</code>',
                'Lorem ipsum',
            ],
            [
                '<dd class="foo">Lorem ipsum</dd>',
                'Lorem ipsum',
            ],
            [
                '<del class="foo">Lorem ipsum</del>',
                '<del>Lorem ipsum</del>',
            ],
            [
                '<div class="foo">Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
            ],
            [
                '<dl class="foo">Lorem ipsum</dl>',
                'Lorem ipsum',
            ],
            [
                '<dt class="foo">Lorem ipsum</dt>',
                'Lorem ipsum',
            ],
            [
                '<em class="foo">Lorem ipsum</em>',
                '<em>Lorem ipsum</em>',
            ],
            [
                '<figcaption class="foo">Lorem ipsum</figcaption>',
                '<figcaption>Lorem ipsum</figcaption>',
            ],
            [
                '<figure class="foo">Lorem ipsum</figure>',
                '<figure>Lorem ipsum</figure>',
            ],
            [
                '<h1 class="foo">Lorem ipsum</h1>',
                '<h1>Lorem ipsum</h1>',
            ],
            [
                '<h2 class="foo">Lorem ipsum</h2>',
                '<h2>Lorem ipsum</h2>',
            ],
            [
                '<h3 class="foo">Lorem ipsum</h3>',
                '<h3>Lorem ipsum</h3>',
            ],
            [
                '<h4 class="foo">Lorem ipsum</h4>',
                '<h4>Lorem ipsum</h4>',
            ],
            [
                '<h5 class="foo">Lorem ipsum</h5>',
                '<h5>Lorem ipsum</h5>',
            ],
            [
                '<h6 class="foo">Lorem ipsum</h6>',
                '<h6>Lorem ipsum</h6>',
            ],
            [
                '<hr class="foo" />',
                '',
            ],
            [
                '<iframe src="/frame/example" width="300" height="300" frameborder="0">Lorem ipsum</iframe>',
                'Lorem ipsum',
            ],
            [
                '<iframe>Lorem ipsum</iframe>',
                'Lorem ipsum',
            ],
            [
                '<img src="/img/example.jpg" alt="Image alternative text" title="Image title" class="foo" />',
                '',
            ],
            [
                '<img src="http://trusted.com/img/example.jpg" alt="Image alternative text" title="Image title" class="foo" />',
                '',
            ],
            [
                '<img />',
                '',
            ],
            [
                '<i class="foo">Lorem ipsum</i>',
                '<i>Lorem ipsum</i>',
            ],
            [
                '<li class="foo">Lorem ipsum</li>',
                'Lorem ipsum',
            ],
            [
                '<ol class="foo">Lorem ipsum</ol>',
                'Lorem ipsum',
            ],
            [
                '<p class="foo">Lorem ipsum</p>',
                '<p>Lorem ipsum</p>',
            ],
            [
                '<pre class="foo">Lorem ipsum</pre>',
                'Lorem ipsum',
            ],
            [
                '<q class="foo">Lorem ipsum</q>',
                '<q>Lorem ipsum</q>',
            ],
            [
                '<rp class="foo">Lorem ipsum</rp>',
                'Lorem ipsum',
            ],
            [
                '<rt class="foo">Lorem ipsum</rt>',
                'Lorem ipsum',
            ],
            [
                '<ruby class="foo">Lorem ipsum</ruby>',
                'Lorem ipsum',
            ],
            [
                '<small class="foo">Lorem ipsum</small>',
                '<small>Lorem ipsum</small>',
            ],
            [
                '<span class="foo">Lorem ipsum</span>',
                '<span>Lorem ipsum</span>',
            ],
            [
                '<strong class="foo">Lorem ipsum</strong>',
                '<strong>Lorem ipsum</strong>',
            ],
            [
                '<b class="foo">Lorem ipsum</b>',
                '<strong>Lorem ipsum</strong>',
            ],
            [
                '<sub class="foo">Lorem ipsum</sub>',
                '<sub>Lorem ipsum</sub>',
            ],
            [
                '<sup class="foo">Lorem ipsum</sup>',
                '<sup>Lorem ipsum</sup>',
            ],
            [
                '<table class="foo">Lorem ipsum</table>',
                'Lorem ipsum',
            ],
            [
                '<tbody class="foo">Lorem ipsum</tbody>',
                'Lorem ipsum',
            ],
            [
                '<td class="foo">Lorem ipsum</td>',
                'Lorem ipsum',
            ],
            [
                '<tfoot class="foo">Lorem ipsum</tfoot>',
                'Lorem ipsum',
            ],
            [
                '<thead class="foo">Lorem ipsum</thead>',
                'Lorem ipsum',
            ],
            [
                '<th class="foo">Lorem ipsum</th>',
                'Lorem ipsum',
            ],
            [
                '<tr class="foo">Lorem ipsum</tr>',
                'Lorem ipsum',
            ],
            [
                '<ul class="foo">Lorem ipsum</ul>',
                'Lorem ipsum',
            ],

            /*
             * Links
             */

            [
                '<a href="mailto:test&#64;gmail.com">Test</a>',
                '<a href="mailto:test&#64;gmail.com">Test</a>',
            ],
            [
                '<a href="mailto:alert(\'ok\')">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="javascript:alert(\'ok\')">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="javascript://%0Aalert(document.cookie)">Test</a>',
                '<a>Test</a>',
            ],
            [
                '<a href="http://untrusted.com" onclick="alert(\'ok\')">Test</a>',
                '<a href="http://untrusted.com">Test</a>',
            ],
            [
                '<a href="https://trusted.com">Test</a>',
                '<a href="https://trusted.com">Test</a>',
            ],
            [
                '<a>Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],
            [
                '<a href="&#106;&#97;&#118;&#97;&#115;&#99;&#114;&#105;&#112;&#116;&#58;&#97;&#108;&#101;&#114;&#116;&#40;&#39;&#88;&#83;&#83;&#39;&#41;">Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],
            [
                '<a href="http://trusted.com/index.html#this:stuff">Lorem ipsum</a>',
                '<a href="http://trusted.com/index.html#this:stuff">Lorem ipsum</a>',
            ],
            [
                '<a href="java\0&#14;\t\r\n script:alert(\\\'foo\\\')">Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],
            [
                '<a href= onmouseover="alert(\\\'XSS\\\');">Lorem ipsum</a>',
                '<a>Lorem ipsum</a>',
            ],


            /*
             * Scripts
             */

            [
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.<script>alert(\'ok\');</script></div>',
                '<div>Lorem ipsum dolor sit amet, consectetur adipisicing elit.</div>',
            ],
            [
                '<figure><img src="/img/example.jpg" onclick="alert(\'ok\')" /></figure>',
                '<figure></figure>',
            ],
            [
                '<a href="javascript:alert(\'ok\')">Lorem ipsum dolor sit amet, consectetur adipisicing elit.</a>',
                '<a>Lorem ipsum dolor sit amet, consectetur adipisicing elit.</a>',
            ],
            [
                '"><script>...</script><input value="',
                '&#34;&gt;',
            ],
            [
                '<scr<script>ipt>alert(1)</script>',
                '',
            ],
            [
                '<scr<a>ipt>alert(1)</script>',
                '<a>ipt&gt;alert(1)</a>',
            ],

            /*
             * Styles
             */

            [
                '<div>Lorem ipsum dolor sit amet, consectetur.<style>body { background: red; }</style></div>',
                '<div>Lorem ipsum dolor sit amet, consectetur.</div>',
            ],
            [
                '<img src="/img/example.jpg" style="position:absolute;top:0;left:0;width:9000px;height:9000px;" />',
                '',
            ],
            [
                '<a style="font-size: 40px; color: red;">Lorem ipsum dolor sit amet, consectetur.</a>',
                '<a>Lorem ipsum dolor sit amet, consectetur.</a>',
            ],

            /*
             * Ideas extracted from https://github.com/OWASP/java-html-sanitizer
             */

            [
                '<p/b onload=""/',
                '<p></p>',
            ],
            [
                '<p onload=""/b/',
                '<p></p>',
            ],
            [
                '<p onload=""<a href="https://trusted.com/" onload="">first part of the text</> second part',
                '<p><a href="https://trusted.com/">first part of the text second part</a></p>',
            ],
            [
                '<p onload=""<b onload="">Hello',
                '<p><strong>Hello</strong></p>',
            ],
            [
                '<p>Hello world</b style="width:expression(alert(1))">',
                '<p>Hello world</p>',
            ],
            [
                "<A TITLE=\"harmless\0  SCRIPT = javascript:alert(1) ignored = ignored\">",
                '<a title="harmless  SCRIPT &#61; javascript:alert(1) ignored &#61; ignored"></a>',
            ],
            [
                '<div style1="expression(\'alert(1)">Hello</div>',
                '<div>Hello</div>',
            ],
            [
                '<a title="``onmouseover=alert(1337)">Hello</a>',
                '<a title="&#96;&#96;onmouseover&#61;alert(1337) ">Hello</a>',
            ],

        ]);
    }
}

<?php

/*
 * This file is part of the HTML sanitizer project.
 *
 * (c) Titouan Galopin <galopintitouan@gmail.com>
 *
 * For the full copyright and license information, please view the LICENSE
 * file that was distributed with this source code.
 */

namespace Tests\HtmlSanitizer\UrlParser;

use HtmlSanitizer\UrlParser\UrlParser;
use PHPUnit\Framework\TestCase;

class UrlParserTest extends TestCase
{
    /**
     * @dataProvider provideUrls
     */
    public function testParse(string $url, ?array $expected)
    {
        $parsed = (new UrlParser())->parse($url);

        if ($expected === null) {
            $this->assertNull($parsed);
        } else {
            $this->assertInternalType('array', $parsed);
            $this->assertArrayHasKey('scheme', $parsed);
            $this->assertArrayHasKey('host', $parsed);
            $this->assertSame($expected['scheme'], $parsed['scheme']);
            $this->assertSame($expected['host'], $parsed['host']);
        }
    }

    public function provideUrls(): iterable
    {
        $urls = [
            // Simple tests
            'https://trusted.com/link.php' => ['scheme' => 'https', 'host' => 'trusted.com'],
            'https://trusted.com/link.php?query=1#foo' => ['scheme' => 'https', 'host' => 'trusted.com'],
            'https://subdomain.trusted.com/link' => ['scheme' => 'https', 'host' => 'subdomain.trusted.com'],
            '//trusted.com/link.php' => ['scheme' => null, 'host' => 'trusted.com'],
            'https:trusted.com/link.php' => ['scheme' => 'https', 'host' => null],
            'https://untrusted.com/link' => ['scheme' => 'https', 'host' => 'untrusted.com'],

            // Ensure https://bugs.php.net/bug.php?id=73192 is handled
            'https://untrusted.com:80?@trusted.com/' => ['scheme' => 'https', 'host' => 'untrusted.com'],
            'https://untrusted.com:80#@trusted.com/' => ['scheme' => 'https', 'host' => 'untrusted.com'],

            // Ensure https://medium.com/secjuice/php-ssrf-techniques-9d422cb28d51 is handled
            '0://untrusted.com;trusted.com' => null,
            '0://untrusted.com:80;trusted.com:80' => null,
            '0://untrusted.com:80,trusted.com:80' => null,

            // Data-URI
            'data:text/plain;base64,SSBsb3ZlIFBIUAo' => ['scheme' => 'data', 'host' => null],
            'data:text/plain;base64,SSBsb3ZlIFBIUAo=trusted.com' => ['scheme' => 'data', 'host' => null],
            'data:http://trusted.com' => ['scheme' => 'data', 'host' => null],
            'data://text/plain;base64,SSBsb3ZlIFBIUAo=trusted.com' => ['scheme' => 'data', 'host' => 'text'],
            'data://image/png;base64,SSBsb3ZlIFBIUAo=trusted.com' => ['scheme' => 'data', 'host' => 'image'],
            'data:google.com/plain;base64,SSBsb3ZlIFBIUAo=' => ['scheme' => 'data', 'host' => null],
            'data://google.com/plain;base64,SSBsb3ZlIFBIUAo=' => ['scheme' => 'data', 'host' => 'google.com'],

            // Inspired by https://github.com/punkave/sanitize-html/blob/master/test/test.js
            "java\0&#14;\t\r\n script:alert(\'foo\')" => null,
            'java&#0000001script:alert(\\\'foo\\\')' => null,
            'java&#0000000script:alert(\\\'foo\\\')' => null,
            'java<!-- -->script:alert(\'foo\')' => null,

            // Extracted from https://github.com/web-platform-tests/wpt/blob/master/url/resources/urltestdata.json
            "http://example	.\norg" => null,
            'http://user:pass@foo:21/bar;par?b#c' => ['scheme' => 'http', 'host' => 'foo'],
            'https://trusted.com:@untrusted.com' => ['scheme' => 'https', 'host' => 'untrusted.com'],
            'https://:@untrusted.com' => ['scheme' => 'https', 'host' => 'untrusted.com'],
            'non-special://test:@untrusted.com/x' => ['scheme' => 'non-special', 'host' => 'untrusted.com'],
            'non-special://:@untrusted.com/x' => ['scheme' => 'non-special', 'host' => 'untrusted.com'],
            'http:foo.com' => ['scheme' => 'http', 'host' => null],
            "	   :foo.com   \n" => null,
            ' foo.com  ' => ['scheme' => null, 'host' => null],
            'a:	 foo.com' => null,
            'http://f:21/ b ? d # e ' => ['scheme' => 'http', 'host' => 'f'],
            'lolscheme:x x#x x' => ['scheme' => 'lolscheme', 'host' => null],
            'http://f:/c' => ['scheme' => 'http', 'host' => 'f'],
            'http://f:0/c' => ['scheme' => 'http', 'host' => 'f'],
            'http://f:00000000000000/c' => ['scheme' => 'http', 'host' => 'f'],
            'http://f:00000000000000000000080/c' => ['scheme' => 'http', 'host' => 'f'],
            "http://f:\n/c" => null,
            '' => null,
            '  	' => null,
            ':foo.com/' => null,
            ':foo.com\\' => null,
            ':' => null,
            ':a' => null,
            ':/' => null,
            ':\\' => null,
            ':#' => null,
            '#' => ['scheme' => null, 'host' => null],
            '#/' => ['scheme' => null, 'host' => null],
            '#\\' => ['scheme' => null, 'host' => null],
            '#;?' => ['scheme' => null, 'host' => null],
            '?' => ['scheme' => null, 'host' => null],
            '/' => ['scheme' => null, 'host' => null],
            ':23' => null,
            '/:23' => ['scheme' => null, 'host' => null],
            '::' => null,
            '::23' => null,
            'foo://' => ['scheme' => 'foo', 'host' => ''],
            'http://a:b@c:29/d' => ['scheme' => 'http', 'host' => 'c'],
            'http::@c:29' => ['scheme' => 'http', 'host' => null],
            'http://&a:foo(b]c@d:2/' => ['scheme' => 'http', 'host' => 'd'],
            'http://::@c@d:2' => null,
            'http://foo.com:b@d/' => ['scheme' => 'http', 'host' => 'd'],
            'http://foo.com/\\@' => ['scheme' => 'http', 'host' => 'foo.com'],
            'http:\\foo.com\\' => ['scheme' => 'http', 'host' => null],
            'http:\\a\\b:c\\d@foo.com\\' => ['scheme' => 'http', 'host' => null],
            'foo:/' => ['scheme' => 'foo', 'host' => null],
            'foo:/bar.com/' => ['scheme' => 'foo', 'host' => null],
            'foo://///////' => ['scheme' => 'foo', 'host' => ''],
            'foo://///////bar.com/' => ['scheme' => 'foo', 'host' => ''],
            'foo:////://///' => ['scheme' => 'foo', 'host' => ''],
            'c:/foo' => ['scheme' => 'c', 'host' => null],
            '//foo/bar' => ['scheme' => null, 'host' => 'foo'],
            'http://foo/path;a??e#f#g' => ['scheme' => 'http', 'host' => 'foo'],
            'http://foo/abcd?efgh?ijkl' => ['scheme' => 'http', 'host' => 'foo'],
            'http://foo/abcd#foo?bar' => ['scheme' => 'http', 'host' => 'foo'],
            '[61:24:74]:98' => null,
            'http:[61:27]/:foo' => ['scheme' => 'http', 'host' => null],
            'http://[2001::1]' => ['scheme' => 'http', 'host' => '[2001::1]'],
            'http://[::127.0.0.1]' => ['scheme' => 'http', 'host' => '[::127.0.0.1]'],
            'http://[0:0:0:0:0:0:13.1.68.3]' => ['scheme' => 'http', 'host' => '[0:0:0:0:0:0:13.1.68.3]'],
            'http://[2001::1]:80' => ['scheme' => 'http', 'host' => '[2001::1]'],
            'http:/example.com/' => ['scheme' => 'http', 'host' => null],
            'ftp:/example.com/' => ['scheme' => 'ftp', 'host' => null],
            'https:/example.com/' => ['scheme' => 'https', 'host' => null],
            'madeupscheme:/example.com/' => ['scheme' => 'madeupscheme', 'host' => null],
            'file:/example.com/' => ['scheme' => 'file', 'host' => null],
            'ftps:/example.com/' => ['scheme' => 'ftps', 'host' => null],
            'gopher:/example.com/' => ['scheme' => 'gopher', 'host' => null],
            'ws:/example.com/' => ['scheme' => 'ws', 'host' => null],
            'wss:/example.com/' => ['scheme' => 'wss', 'host' => null],
            'data:/example.com/' => ['scheme' => 'data', 'host' => null],
            'javascript:/example.com/' => ['scheme' => 'javascript', 'host' => null],
            'mailto:/example.com/' => ['scheme' => 'mailto', 'host' => null],
            'http:example.com/' => ['scheme' => 'http', 'host' => null],
            'ftp:example.com/' => ['scheme' => 'ftp', 'host' => null],
            'https:example.com/' => ['scheme' => 'https', 'host' => null],
            'madeupscheme:example.com/' => ['scheme' => 'madeupscheme', 'host' => null],
            'ftps:example.com/' => ['scheme' => 'ftps', 'host' => null],
            'gopher:example.com/' => ['scheme' => 'gopher', 'host' => null],
            'ws:example.com/' => ['scheme' => 'ws', 'host' => null],
            'wss:example.com/' => ['scheme' => 'wss', 'host' => null],
            'data:example.com/' => ['scheme' => 'data', 'host' => null],
            'javascript:example.com/' => ['scheme' => 'javascript', 'host' => null],
            'mailto:example.com/' => ['scheme' => 'mailto', 'host' => null],
            '/a/b/c' => ['scheme' => null, 'host' => null],
            '/a/ /c' => ['scheme' => null, 'host' => null],
            '/a%2fc' => ['scheme' => null, 'host' => null],
            '/a/%2f/c' => ['scheme' => null, 'host' => null],
            '#β' => ['scheme' => null, 'host' => null],
            'data:text/html,test#test' => ['scheme' => 'data', 'host' => null],
            'tel:1234567890' => ['scheme' => 'tel', 'host' => null],
            'ssh://example.com/foo/bar.git' => ['scheme' => 'ssh', 'host' => 'example.com'],
            "file:c:\foo\bar.html" => null,
            '  File:c|////foo\\bar.html' => null,
            'C|/foo/bar' => ['scheme' => null, 'host' => null],
            "/C|\foo\bar" => null,
            '//C|/foo/bar' => null,
            '//server/file' => ['scheme' => null, 'host' => 'server'],
            "\\server\file" => null,
            '/\\server/file' => ['scheme' => null, 'host' => null],
            'file:///foo/bar.txt' => ['scheme' => 'file', 'host' => ''],
            'file:///home/me' => ['scheme' => 'file', 'host' => ''],
            '//' => ['scheme' => null, 'host' => ''],
            '///' => ['scheme' => null, 'host' => ''],
            '///test' => ['scheme' => null, 'host' => ''],
            'file://test' => ['scheme' => 'file', 'host' => 'test'],
            'file://localhost' => ['scheme' => 'file', 'host' => 'localhost'],
            'file://localhost/' => ['scheme' => 'file', 'host' => 'localhost'],
            'file://localhost/test' => ['scheme' => 'file', 'host' => 'localhost'],
            'test' => ['scheme' => null, 'host' => null],
            'file:test' => ['scheme' => 'file', 'host' => null],
            'http://example.com/././foo' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/./.foo' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/.' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/./' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/bar/..' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/bar/../' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/..bar' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/bar/../ton' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/bar/../ton/../../a' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/../../..' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/../../../ton' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/%2e' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/%2e%2' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/%2e./%2e%2e/.%2e/%2e.bar' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com////../..' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/bar//../..' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo/bar//..' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/%20foo' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo%' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo%2' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo%2zbar' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo%2Â©zbar' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo%41%7a' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo	%91' => null,
            'http://example.com/foo%00%51' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/(%28:%3A%29)' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/%3A%3a%3C%3c' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/foo	bar' => null,
            'http://example.com\\foo\\bar' => null,
            'http://example.com/%7Ffp3%3Eju%3Dduvgw%3Dd' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/@asdf%40' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/你好你好' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/‥/foo' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/﻿/foo' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://example.com/‮/foo/‭/bar' => ['scheme' => 'http', 'host' => 'example.com'],
            'http://www.google.com/foo?bar=baz#' => ['scheme' => 'http', 'host' => 'www.google.com'],
            'http://www.google.com/foo?bar=baz# »' => ['scheme' => 'http', 'host' => 'www.google.com'],
            'data:test# »' => ['scheme' => 'data', 'host' => null],
            'http://www.google.com' => ['scheme' => 'http', 'host' => 'www.google.com'],
            'http://192.0x00A80001' => ['scheme' => 'http', 'host' => '192.0x00A80001'],
            'http://www/foo%2Ehtml' => ['scheme' => 'http', 'host' => 'www'],
            'http://www/foo/%2E/html' => ['scheme' => 'http', 'host' => 'www'],
            'http://%25DOMAIN:foobar@foodomain.com/' => ['scheme' => 'http', 'host' => 'foodomain.com'],
            "http:\\www.google.com\foo" => null,
            'http://foo:80/' => ['scheme' => 'http', 'host' => 'foo'],
            'http://foo:81/' => ['scheme' => 'http', 'host' => 'foo'],
            'httpa://foo:80/' => ['scheme' => 'httpa', 'host' => 'foo'],
            'https://foo:443/' => ['scheme' => 'https', 'host' => 'foo'],
            'https://foo:80/' => ['scheme' => 'https', 'host' => 'foo'],
            'ftp://foo:21/' => ['scheme' => 'ftp', 'host' => 'foo'],
            'ftp://foo:80/' => ['scheme' => 'ftp', 'host' => 'foo'],
            'gopher://foo:70/' => ['scheme' => 'gopher', 'host' => 'foo'],
            'gopher://foo:443/' => ['scheme' => 'gopher', 'host' => 'foo'],
            'ws://foo:80/' => ['scheme' => 'ws', 'host' => 'foo'],
            'ws://foo:81/' => ['scheme' => 'ws', 'host' => 'foo'],
            'ws://foo:443/' => ['scheme' => 'ws', 'host' => 'foo'],
            'ws://foo:815/' => ['scheme' => 'ws', 'host' => 'foo'],
            'wss://foo:80/' => ['scheme' => 'wss', 'host' => 'foo'],
            'wss://foo:81/' => ['scheme' => 'wss', 'host' => 'foo'],
            'wss://foo:443/' => ['scheme' => 'wss', 'host' => 'foo'],
            'wss://foo:815/' => ['scheme' => 'wss', 'host' => 'foo'],
            'http:@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:/@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http://@www.example.com' => ['scheme' => 'http', 'host' => 'www.example.com'],
            'http:a:b@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:/a:b@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http://a:b@www.example.com' => ['scheme' => 'http', 'host' => 'www.example.com'],
            'http://@pple.com' => ['scheme' => 'http', 'host' => 'pple.com'],
            'http::b@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:/:b@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http://:b@www.example.com' => ['scheme' => 'http', 'host' => 'www.example.com'],
            'http:a:@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:/a:@www.example.com' => ['scheme' => 'http', 'host' => null],
            'http://a:@www.example.com' => ['scheme' => 'http', 'host' => 'www.example.com'],
            'http://www.@pple.com' => ['scheme' => 'http', 'host' => 'pple.com'],
            'http://:@www.example.com' => ['scheme' => 'http', 'host' => 'www.example.com'],
            '/test.txt' => ['scheme' => null, 'host' => null],
            '.' => ['scheme' => null, 'host' => null],
            '..' => ['scheme' => null, 'host' => null],
            'test.txt' => ['scheme' => null, 'host' => null],
            './test.txt' => ['scheme' => null, 'host' => null],
            '../test.txt' => ['scheme' => null, 'host' => null],
            '../aaa/test.txt' => ['scheme' => null, 'host' => null],
            '../../test.txt' => ['scheme' => null, 'host' => null],
            '中/test.txt' => ['scheme' => null, 'host' => null],
            'http://www.example2.com' => ['scheme' => 'http', 'host' => 'www.example2.com'],
            '//www.example2.com' => ['scheme' => null, 'host' => 'www.example2.com'],
            'file:...' => ['scheme' => 'file', 'host' => null],
            'file:..' => ['scheme' => 'file', 'host' => null],
            'file:a' => ['scheme' => 'file', 'host' => null],
            'http://ExAmPlE.CoM' => ['scheme' => 'http', 'host' => 'ExAmPlE.CoM'],
            'http://GOO​⁠﻿goo.com' => ['scheme' => 'http', 'host' => 'GOO​⁠﻿goo.com'],
            'http://www.foo。bar.com' => ['scheme' => 'http', 'host' => 'www.foo。bar.com'],
            'https://x/�?�#�' => ['scheme' => 'https', 'host' => 'x'],
            'http://Ｇｏ.com' => ['scheme' => 'http', 'host' => 'Ｇｏ.com'],
            'http://你好你好' => ['scheme' => 'http', 'host' => '你好你好'],
            'https://faß.ExAmPlE/' => ['scheme' => 'https', 'host' => 'faß.ExAmPlE'],
            'sc://faß.ExAmPlE/' => ['scheme' => 'sc', 'host' => 'faß.ExAmPlE'],
            'http://%30%78%63%30%2e%30%32%35%30.01' => ['scheme' => 'http', 'host' => '%30%78%63%30%2e%30%32%35%30.01'],
            'http://%30%78%63%30%2e%30%32%35%30.01%2e' => ['scheme' => 'http', 'host' => '%30%78%63%30%2e%30%32%35%30.01%2e'],
            'http://０Ｘｃ０．０２５０．０１' => ['scheme' => 'http', 'host' => '０Ｘｃ０．０２５０．０１'],
            'http://./' => ['scheme' => 'http', 'host' => '.'],
            'http://../' => ['scheme' => 'http', 'host' => '..'],
            'http://0..0x300/' => ['scheme' => 'http', 'host' => '0..0x300'],
            'http://foo:💩@example.com/bar' => ['scheme' => 'http', 'host' => 'example.com'],
            '#x' => ['scheme' => null, 'host' => null],
            'https://@test@test@example:800/' => null,
            'https://@@@example' => null,
            'http://`{}:`{}@h/`{}?`{}' => ['scheme' => 'http', 'host' => 'h'],
            'http://host/?\'' => ['scheme' => 'http', 'host' => 'host'],
            'notspecial://host/?\'' => ['scheme' => 'notspecial', 'host' => 'host'],
            '/some/path' => ['scheme' => null, 'host' => null],
            'i' => ['scheme' => null, 'host' => null],
            '../i' => ['scheme' => null, 'host' => null],
            '/i' => ['scheme' => null, 'host' => null],
            '?i' => ['scheme' => null, 'host' => null],
            '#i' => ['scheme' => null, 'host' => null],
            'about:/../' => ['scheme' => 'about', 'host' => null],
            'data:/../' => ['scheme' => 'data', 'host' => null],
            'javascript:/../' => ['scheme' => 'javascript', 'host' => null],
            'mailto:/../' => ['scheme' => 'mailto', 'host' => null],
            'sc://ñ.test/' => ['scheme' => 'sc', 'host' => 'ñ.test'],
            'sc://!"$&\'()*+,-.;<=>^_`{|}~/' => null,
            'sc://%/' => null,
            'x' => ['scheme' => null, 'host' => null],
            'sc:\\../' => ['scheme' => 'sc', 'host' => null],
            'sc::a@example.net' => ['scheme' => 'sc', 'host' => null],
            'wow:%NBD' => ['scheme' => 'wow', 'host' => null],
            'wow:%1G' => ['scheme' => 'wow', 'host' => null],
            'ftp://%e2%98%83' => ['scheme' => 'ftp', 'host' => '%e2%98%83'],
            'https://%e2%98%83' => ['scheme' => 'https', 'host' => '%e2%98%83'],
            'http://127.0.0.1:10100/relative_import.html' => ['scheme' => 'http', 'host' => '127.0.0.1'],
            'http://facebook.com/?foo=%7B%22abc%22' => ['scheme' => 'http', 'host' => 'facebook.com'],
            'https://localhost:3000/jqueryui@1.2.3' => ['scheme' => 'https', 'host' => 'localhost'],
            '?a=b&c=d' => ['scheme' => null, 'host' => null],
            '??a=b&c=d' => ['scheme' => null, 'host' => null],
            'http:' => ['scheme' => 'http', 'host' => null],
            'sc:' => ['scheme' => 'sc', 'host' => null],
            'http://foo.bar/baz?qux#fobar' => ['scheme' => 'http', 'host' => 'foo.bar'],
            'http://foo.bar/baz?qux#foo"bar' => ['scheme' => 'http', 'host' => 'foo.bar'],
            'http://foo.bar/baz?qux#foo<bar' => ['scheme' => 'http', 'host' => 'foo.bar'],
            'http://foo.bar/baz?qux#foo>bar' => ['scheme' => 'http', 'host' => 'foo.bar'],
            'http://foo.bar/baz?qux#foo`bar' => ['scheme' => 'http', 'host' => 'foo.bar'],
            'http://192.168.257' => ['scheme' => 'http', 'host' => '192.168.257'],
            'http://192.168.257.com' => ['scheme' => 'http', 'host' => '192.168.257.com'],
            'http://256' => ['scheme' => 'http', 'host' => '256'],
            'http://256.com' => ['scheme' => 'http', 'host' => '256.com'],
            'http://999999999' => ['scheme' => 'http', 'host' => '999999999'],
            'http://999999999.com' => ['scheme' => 'http', 'host' => '999999999.com'],
            'http://10000000000.com' => ['scheme' => 'http', 'host' => '10000000000.com'],
            'http://4294967295' => ['scheme' => 'http', 'host' => '4294967295'],
            'http://0xffffffff' => ['scheme' => 'http', 'host' => '0xffffffff'],
            'http://256.256.256.256.256' => ['scheme' => 'http', 'host' => '256.256.256.256.256'],
            'https://0x.0x.0' => ['scheme' => 'https', 'host' => '0x.0x.0'],
            'file:///C%3A/' => ['scheme' => 'file', 'host' => ''],
            'file:///C%7C/' => ['scheme' => 'file', 'host' => ''],
            'pix/submit.gif' => ['scheme' => null, 'host' => null],
            '//d:' => ['scheme' => null, 'host' => 'd'],
            '//d:/..' => ['scheme' => null, 'host' => 'd'],
            'file:' => ['scheme' => 'file', 'host' => null],
            '?x' => ['scheme' => null, 'host' => null],
            'file:?x' => ['scheme' => 'file', 'host' => null],
            'file:#x' => ['scheme' => 'file', 'host' => null],
            'file:\\//' => ['scheme' => 'file', 'host' => null],
            'file:\\\\' => ['scheme' => 'file', 'host' => null],
            'file:\\\\?fox' => ['scheme' => 'file', 'host' => null],
            'file:\\\\#guppy' => ['scheme' => 'file', 'host' => null],
            'file://spider///' => ['scheme' => 'file', 'host' => 'spider'],
            'file:\\localhost//' => ['scheme' => 'file', 'host' => null],
            'file:///localhost//cat' => ['scheme' => 'file', 'host' => ''],
            'file://\\/localhost//cat' => null,
            'file://localhost//a//../..//' => ['scheme' => 'file', 'host' => 'localhost'],
            '/////mouse' => ['scheme' => null, 'host' => ''],
            '\\//pig' => ['scheme' => null, 'host' => null],
            '\\/localhost//pig' => ['scheme' => null, 'host' => null],
            '//localhost//pig' => ['scheme' => null, 'host' => 'localhost'],
            '/..//localhost//pig' => ['scheme' => null, 'host' => null],
            'file://' => ['scheme' => 'file', 'host' => ''],
            '/rooibos' => ['scheme' => null, 'host' => null],
            '/?chai' => ['scheme' => null, 'host' => null],
            'C|' => ['scheme' => null, 'host' => null],
            'C|#' => ['scheme' => null, 'host' => null],
            'C|?' => ['scheme' => null, 'host' => null],
            'C|/' => ['scheme' => null, 'host' => null],
            "C|\n/" => null,
            'C|\\' => ['scheme' => null, 'host' => null],
            'C' => ['scheme' => null, 'host' => null],
            'C|a' => ['scheme' => null, 'host' => null],
            '/c:/foo/bar' => ['scheme' => null, 'host' => null],
            '/c|/foo/bar' => ['scheme' => null, 'host' => null],
            "file:\c:\foo\bar" => null,
            'file://example.net/C:/' => ['scheme' => 'file', 'host' => 'example.net'],
            'file://1.2.3.4/C:/' => ['scheme' => 'file', 'host' => '1.2.3.4'],
            'file://[1::8]/C:/' => ['scheme' => 'file', 'host' => '[1::8]'],
            'file:/C|/' => ['scheme' => 'file', 'host' => null],
            'file://C|/' => null,
            'file:?q=v' => ['scheme' => 'file', 'host' => null],
            'file:#frag' => ['scheme' => 'file', 'host' => null],
            'http://[1:0::]' => ['scheme' => 'http', 'host' => '[1:0::]'],
            'sc://ñ' => ['scheme' => 'sc', 'host' => 'ñ'],
            'sc://ñ?x' => ['scheme' => 'sc', 'host' => 'ñ'],
            'sc://ñ#x' => ['scheme' => 'sc', 'host' => 'ñ'],
            'sc://?' => ['scheme' => 'sc', 'host' => ''],
            'sc://#' => ['scheme' => 'sc', 'host' => ''],
            '////' => ['scheme' => null, 'host' => ''],
            '////x/' => ['scheme' => null, 'host' => ''],
            'tftp://foobar.com/someconfig;mode=netascii' => ['scheme' => 'tftp', 'host' => 'foobar.com'],
            'telnet://user:pass@foobar.com:23/' => ['scheme' => 'telnet', 'host' => 'foobar.com'],
            'ut2004://10.10.10.10:7777/Index.ut2' => null,
            'redis://foo:bar@somehost:6379/0?baz=bam&qux=baz' => ['scheme' => 'redis', 'host' => 'somehost'],
            'rsync://foo@host:911/sup' => ['scheme' => 'rsync', 'host' => 'host'],
            'git://github.com/foo/bar.git' => ['scheme' => 'git', 'host' => 'github.com'],
            'irc://myserver.com:6999/channel?passwd' => ['scheme' => 'irc', 'host' => 'myserver.com'],
            'dns://fw.example.org:9999/foo.bar.org?type=TXT' => ['scheme' => 'dns', 'host' => 'fw.example.org'],
            'ldap://localhost:389/ou=People,o=JNDITutorial' => ['scheme' => 'ldap', 'host' => 'localhost'],
            'git+https://github.com/foo/bar' => ['scheme' => 'git+https', 'host' => 'github.com'],
            'urn:ietf:rfc:2648' => ['scheme' => 'urn', 'host' => null],
            'tag:joe@example.org,2001:foo/bar' => ['scheme' => 'tag', 'host' => null],
            'non-special://%E2%80%A0/' => ['scheme' => 'non-special', 'host' => '%E2%80%A0'],
            'non-special://H%4fSt/path' => ['scheme' => 'non-special', 'host' => 'H%4fSt'],
            'non-special://[1:2:0:0:5:0:0:0]/' => ['scheme' => 'non-special', 'host' => '[1:2:0:0:5:0:0:0]'],
            'non-special://[1:2:0:0:0:0:0:3]/' => ['scheme' => 'non-special', 'host' => '[1:2:0:0:0:0:0:3]'],
            'non-special://[1:2::3]:80/' => ['scheme' => 'non-special', 'host' => '[1:2::3]'],
            'blob:https://example.com:443/' => ['scheme' => 'blob', 'host' => null],
            'blob:d3958f5c-0777-0845-9dcf-2cb28783acaf' => ['scheme' => 'blob', 'host' => null],
            'http://0177.0.0.0189' => ['scheme' => 'http', 'host' => '0177.0.0.0189'],
            'http://0x7f.0.0.0x7g' => ['scheme' => 'http', 'host' => '0x7f.0.0.0x7g'],
            'http://0X7F.0.0.0X7G' => ['scheme' => 'http', 'host' => '0X7F.0.0.0X7G'],
            'http://[0:1:0:1:0:1:0:1]' => ['scheme' => 'http', 'host' => '[0:1:0:1:0:1:0:1]'],
            'http://[1:0:1:0:1:0:1:0]' => ['scheme' => 'http', 'host' => '[1:0:1:0:1:0:1:0]'],
            'http://example.org/test?"' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?#' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?<' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?>' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?⌣' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?%23%23' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?%GH' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?a#%EF' => ['scheme' => 'http', 'host' => 'example.org'],
            'http://example.org/test?a#%GH' => ['scheme' => 'http', 'host' => 'example.org'],
            'test-a-colon-slash.html' => ['scheme' => null, 'host' => null],
            'test-a-colon-slash-slash.html' => ['scheme' => null, 'host' => null],
            'test-a-colon-slash-b.html' => ['scheme' => null, 'host' => null],
            'test-a-colon-slash-slash-b.html' => ['scheme' => null, 'host' => null],
            'http://example.org/test?a#bc' => ['scheme' => 'http', 'host' => 'example.org'],
            'http:\\/\\/f:b\\/c' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/f: \\/c' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/f:fifty-two\\/c' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/f:999999\\/c' => ['scheme' => 'http', 'host' => null],
            'non-special:\\/\\/f:999999\\/c' => ['scheme' => 'non-special', 'host' => null],
            'http:\\/\\/f: 21 \\/ b ? d # e ' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[1::2]:3:4' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/2001::1' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/2001::1]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/2001::1]:80' => ['scheme' => 'http', 'host' => null],
            'file:\\/\\/example:1\\/' => ['scheme' => 'file', 'host' => null],
            'file:\\/\\/example:test\\/' => ['scheme' => 'file', 'host' => null],
            'file:\\/\\/example%\\/' => ['scheme' => 'file', 'host' => null],
            'file:\\/\\/[example]\\/' => ['scheme' => 'file', 'host' => null],
            'http:\\/\\/user:pass@\\/' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/foo:-80\\/' => ['scheme' => 'http', 'host' => null],
            'http:\\/:@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/user@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'https:@\\/www.example.com' => ['scheme' => 'https', 'host' => null],
            'http:a:b@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/a:b@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/a:b@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http::@\\/www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:@:www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/@:www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/@:www.example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/example example.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/Goo%20 goo%7C|.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[:]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/GOO\\u00a0\\u3000goo.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/\\ufdd0zyx.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/%ef%b7%90zyx.com' => ['scheme' => 'http', 'host' => null],
            'https:\\/\\/\\ufffd' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/%EF%BF%BD' => ['scheme' => 'https', 'host' => null],
            'http:\\/\\/\\uff05\\uff14\\uff11.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/%ef%bc%85%ef%bc%94%ef%bc%91.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/\\uff05\\uff10\\uff10.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/%ef%bc%85%ef%bc%90%ef%bc%90.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/%zz%66%a.com' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/%25' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/hello%00' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/192.168.0.257' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/%3g%78%63%30%2e%30%32%35%30%2E.01' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/192.168.0.1 hello' => ['scheme' => 'http', 'host' => null],
            'https:\\/\\/x x:12' => ['scheme' => 'https', 'host' => null],
            'http:\\/\\/[www.google.com]\\/' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[google.com]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[::1.2.3.4x]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[::1.2.3.]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[::1.2.]' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/[::1.]' => ['scheme' => 'http', 'host' => null],
            '..\\/i' => ['scheme' => null, 'host' => null],
            '\\/i' => ['scheme' => null, 'host' => null],
            'sc:\\/\\/\\u0000\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/ \\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/@\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/te@s:t@\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/:\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/:12\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/[\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/\\\\/' => ['scheme' => 'sc', 'host' => null],
            'sc:\\/\\/]\\/' => ['scheme' => 'sc', 'host' => null],
            'ftp:\\/\\/example.com%80\\/' => ['scheme' => 'ftp', 'host' => null],
            'ftp:\\/\\/example.com%A0\\/' => ['scheme' => 'ftp', 'host' => null],
            'https:\\/\\/example.com%80\\/' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/example.com%A0\\/' => ['scheme' => 'https', 'host' => null],
            'http:\\/\\/10000000000' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/4294967296' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/0xffffffff1' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/256.256.256.256' => ['scheme' => 'http', 'host' => null],
            'https:\\/\\/0x100000000\\/test' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/256.0.0.1\\/test' => ['scheme' => 'https', 'host' => null],
            'http:\\/\\/[0:1:2:3:4:5:6:7:8]' => ['scheme' => 'http', 'host' => null],
            'https:\\/\\/[0::0::0]' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/[0:.0]' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/[0:0:]' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/[0:1:2:3:4:5:6:7.0.0.0.1]' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/[0:1.00.0.0.0]' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/[0:1.290.0.0.0]' => ['scheme' => 'https', 'host' => null],
            'https:\\/\\/[0:1.23.23]' => ['scheme' => 'https', 'host' => null],
            'http:\\/\\/?' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/#' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/f:4294967377\\/c' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/f:18446744073709551697\\/c' => ['scheme' => 'http', 'host' => null],
            'http:\\/\\/f:340282366920938463463374607431768211537\\/c' => ['scheme' => 'http', 'host' => null],
            'non-special:\\/\\/[:80\\/' => ['scheme' => 'non-special', 'host' => null],
            'http:\\/\\/[::127.0.0.0.1]' => ['scheme' => 'http', 'host' => null],
            'a' => ['scheme' => null, 'host' => null],
            'a\\/' => ['scheme' => null, 'host' => null],
            'a\\/\\/' => ['scheme' => null, 'host' => null],
            'test-a-colon.html' => ['scheme' => null, 'host' => null],
            'test-a-colon-b.html' => ['scheme' => null, 'host' => null],
        ];

        foreach ($urls as $url => $expected) {
            yield $url => [$url, $expected];
        }
    }
}

