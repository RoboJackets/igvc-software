#ifndef CUSTOMHIGHLIGHTER_H
#define CUSTOMHIGHLIGHTER_H

#include <QSyntaxHighlighter>
#include <QString>
#include <QRegExp>
#include <QTextCharFormat>
#include <QVector>
#include <QColor>

class CustomHighlighter : public QSyntaxHighlighter
 {
     Q_OBJECT

 public:
     CustomHighlighter(QTextDocument *parent = 0);

 protected:
     void highlightBlock(const QString &text);

 private:
     struct HighlightingRule
     {
         QRegExp pattern;
         QTextCharFormat format;
     };
     QVector<HighlightingRule> highlightingRules;

     QRegExp commentStartExpression;
     QRegExp commentEndExpression;

     QTextCharFormat keywordFormat;
     QTextCharFormat errorFormat;
     QTextCharFormat warningFormat;
     QTextCharFormat infoFormat;
     QTextCharFormat classFormat;
     QTextCharFormat singleLineCommentFormat;
     QTextCharFormat multiLineCommentFormat;
     QTextCharFormat quotationFormat;
     QTextCharFormat functionFormat;
 };

#endif // CUSTOMHIGHLIGHTER_H
