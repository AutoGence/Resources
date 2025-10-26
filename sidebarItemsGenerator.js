// Custom sidebar generator that includes TOC for each document
// This automatically adds H2/H3 headings from each page as nested items in the sidebar

const fs = require('fs');
const path = require('path');
const { slug: githubSlug } = require('github-slugger');

// Extract headings from markdown content
function extractHeadings(content) {
  const headings = [];
  const lines = content.split('\n');
  const seenSlugs = new Set();

  for (const line of lines) {
    // Match markdown headings (## to ####)
    const match = line.match(/^(#{2,4})\s+(.+)$/);
    if (match) {
      const level = match[1].length;
      let text = match[2].trim();
      // Remove markdown formatting like **, __, etc.
      text = text.replace(/\*\*/g, '').replace(/__/g, '').replace(/\*/g, '').replace(/`/g, '');

      // Use github-slugger to create slug (same as Docusaurus)
      let slugValue = githubSlug(text);

      // Handle duplicate slugs by appending -1, -2, etc. (same as github-slugger does)
      let finalSlug = slugValue;
      let counter = 1;
      while (seenSlugs.has(finalSlug)) {
        finalSlug = `${slugValue}-${counter}`;
        counter++;
      }
      seenSlugs.add(finalSlug);

      headings.push({
        level,
        text,
        slug: finalSlug,
      });
    }
  }

  return headings;
}

// Convert flat list of headings to hierarchical structure
// Since Docusaurus sidebar doesn't support nested links directly,
// we'll indent them visually by creating categories for H2/H3
function buildHeadingHierarchy(headings, docPath) {
  const items = [];

  for (let i = 0; i < headings.length; i++) {
    const heading = headings[i];
    const nextHeading = headings[i + 1];

    // Check if this heading has children (next heading has higher level number = deeper nesting)
    const hasChildren = nextHeading && nextHeading.level > heading.level;

    if (hasChildren && heading.level <= 3) {
      // This heading has children - make it a category
      const childItems = [];
      let j = i + 1;

      // Collect all children
      while (j < headings.length && headings[j].level > heading.level) {
        const child = headings[j];

        // Check if this child has its own children
        const childHasChildren = headings[j + 1] && headings[j + 1].level > child.level;

        if (childHasChildren && child.level < 4) {
          // Child is H2 or H3 with children - make it a sub-category
          const grandchildren = [];
          let k = j + 1;

          while (k < headings.length && headings[k].level > child.level) {
            grandchildren.push({
              type: 'link',
              label: headings[k].text,
              href: `/docs/${docPath}#${headings[k].slug}`,
            });
            k++;
            if (k < headings.length && headings[k].level <= child.level) break;
          }

          childItems.push({
            type: 'category',
            label: child.text,
            collapsed: false,
            items: grandchildren,
          });

          j = k;
        } else {
          // Child is a simple link
          childItems.push({
            type: 'link',
            label: child.text,
            href: `/docs/${docPath}#${child.slug}`,
          });
          j++;
        }

        if (j < headings.length && headings[j].level <= heading.level) break;
      }

      items.push({
        type: 'category',
        label: heading.text,
        collapsed: false,
        items: childItems,
      });

      i = j - 1; // Skip processed children
    } else {
      // No children - just a link
      items.push({
        type: 'link',
        label: heading.text,
        href: `/docs/${docPath}#${heading.slug}`,
      });
    }
  }

  return items;
}

// Convert a doc item to include its TOC as nested items
async function addTocToDocItem(item, docsPath, docId, label) {
  try {
    // Construct the file path
    const filePath = path.join(docsPath, `${docId}.md`);

    // Check if file exists
    if (!fs.existsSync(filePath)) {
      console.log(`File not found: ${filePath}`);
      return item;
    }

    // Read the file content
    const content = fs.readFileSync(filePath, 'utf-8');

    // Extract headings (H2, H3, H4)
    const headings = extractHeadings(content);

    console.log(`Document ${docId}: Found ${headings.length} headings`);

    if (headings.length === 0) {
      return item;
    }

    // Handle index files specially - they use the parent directory as the URL
    let docPath = docId;
    if (docId.endsWith('/index')) {
      docPath = docId.replace('/index', '/');
    }

    // Build hierarchical TOC structure
    const tocItems = buildHeadingHierarchy(headings, docPath);

    console.log(`Document ${docId}: Created hierarchical TOC with ${tocItems.length} top-level items`);

    // Return the item as a category with TOC items
    if (tocItems.length > 0) {
      return {
        type: 'category',
        label: label,
        collapsed: false,
        link: {
          type: 'doc',
          id: docId,
        },
        items: tocItems,
      };
    }

    return item;
  } catch (error) {
    console.warn(`Error processing ${docId}:`, error.message);
    return item;
  }
}

// Recursively process sidebar items
async function processSidebarItems(items, docsPath) {
  const processedItems = [];

  for (const item of items) {
    if (typeof item === 'string') {
      // Simple doc reference - convert to category with TOC
      const docId = item;
      const label = item.split('/').pop().replace(/-/g, ' ').replace(/\b\w/g, c => c.toUpperCase());

      const docItem = {
        type: 'doc',
        id: docId,
        label: label,
      };
      const processed = await addTocToDocItem(docItem, docsPath, docId, label);
      processedItems.push(processed);
    } else if (item.type === 'doc') {
      // Doc item - add TOC
      const label = item.label || item.id.split('/').pop().replace(/-/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
      const processed = await addTocToDocItem(item, docsPath, item.id, label);
      processedItems.push(processed);
    } else if (item.type === 'category') {
      // Category - recursively process its items
      const processedCategoryItems = await processSidebarItems(item.items || [], docsPath);

      // If category has a link to a doc, add TOC for that doc too
      if (item.link && item.link.type === 'doc') {
        const docId = item.link.id;
        const filePath = path.join(docsPath, `${docId}.md`);

        try {
          if (fs.existsSync(filePath)) {
            const content = fs.readFileSync(filePath, 'utf-8');
            const headings = extractHeadings(content);

            // Handle index files specially
            let docPath = docId;
            if (docId.endsWith('/index')) {
              docPath = docId.replace('/index', '/');
            }

            // Build hierarchical TOC structure
            const tocItems = buildHeadingHierarchy(headings, docPath);

            // Prepend TOC items to the category items
            processedCategoryItems.unshift(...tocItems);
          }
        } catch (error) {
          console.warn(`Error processing category link ${docId}:`, error.message);
        }
      }

      processedItems.push({
        ...item,
        items: processedCategoryItems,
      });
    } else {
      // Other item types (link, etc.) - keep as is
      processedItems.push(item);
    }
  }

  return processedItems;
}

// Main sidebar generator function
async function sidebarItemsGenerator({
  defaultSidebarItemsGenerator,
  numberPrefixParser,
  item,
  version,
  docs,
  categoriesMetadata,
  isCategoryIndex,
  ...rest
}) {
  // Generate default sidebar items
  const sidebarItems = await defaultSidebarItemsGenerator({
    item,
    version,
    docs,
    categoriesMetadata,
    isCategoryIndex,
    numberPrefixParser,
    ...rest,
  });

  // Get docs path
  const docsPath = path.join(__dirname, 'docs');

  // Process all sidebar items to add TOC
  const processedItems = await processSidebarItems(sidebarItems, docsPath);

  return processedItems;
}

module.exports = sidebarItemsGenerator;
