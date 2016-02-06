#include <stdio.h>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>

#include "bare_panel.h"

namespace bare
{

void BarePanel::subCallback(const std_msgs::UInt8& msg) {
	char str[4];
	sprintf(str, "%u", msg.data);
	output_topic_editor_->setText(str);
}

BarePanel::BarePanel( QWidget* parent )
  : rviz::Panel( parent )
{
	QHBoxLayout* topic_layout = new QHBoxLayout;
	topic_layout->addWidget( new QLabel( "Battery Level:" ) );
	output_topic_editor_ = new QLabel("TEST");
	topic_layout->addWidget( output_topic_editor_ );

	QVBoxLayout* layout = new QVBoxLayout;
	layout->addLayout( topic_layout );
	setLayout( layout );
	output_topic_editor_->setText("COOLBEFORE");

	sub = nh_.subscribe("/battery", 1, &BarePanel::subCallback, this);

	//connect( this, SIGNAL( changeText() ), output_topic_editor_, SLOT( setTextLabel() ));
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bare::BarePanel,rviz::Panel )