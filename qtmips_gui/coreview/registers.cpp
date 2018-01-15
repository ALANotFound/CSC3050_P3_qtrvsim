#include "registers.h"
#include <cmath>

using namespace coreview;

//////////////////////
#define WIDTH 680
#define HEIGHT 30
#define PENW 1
//////////////////////

Registers::Registers() : QGraphicsObject(nullptr) {
    con_read1 = new Connector(-M_PI_2);
    con_read1_reg = new Connector(-M_PI_2);
    con_read2 = new Connector(-M_PI_2);
    con_read2_reg = new Connector(-M_PI_2);
    con_write = new Connector(-M_PI_2);
    con_write_reg = new Connector(-M_PI_2);
    con_ctl_write = new Connector(-M_PI_2);

    // TODO do we want to have any hooks on real registers?

    // TODO add labels for connections

    name = new QGraphicsSimpleTextItem("Registers", this);
    QRectF name_box = name->boundingRect();
    name->setPos(WIDTH/2 - name_box.width()/2, HEIGHT/2 - name_box.height()/2);

    setPos(x(), y()); // set connector's position
}

Registers::~Registers() {
    delete con_read1;
    delete con_read1_reg;
    delete con_read2;
    delete con_read2_reg;
    delete con_write;
    delete con_write_reg;
    delete con_ctl_write;
}

QRectF Registers::boundingRect() const {
    return QRectF(-PENW / 2, -PENW / 2, WIDTH + PENW, HEIGHT + PENW);
}

void Registers::paint(QPainter *painter, const QStyleOptionGraphicsItem *option __attribute((unused)), QWidget *widget __attribute((unused))) {
    painter->drawRect(0, 0, WIDTH, HEIGHT);
    // TODO anything else?
}

void Registers::setPos(qreal x, qreal y) {
    QGraphicsObject::setPos(x, y);

    con_read1_reg->setPos(x + 30, y + HEIGHT);
    con_read2_reg->setPos(x + 40, y + HEIGHT);
    con_read1->setPos(x + 60, y + HEIGHT);
    con_read2->setPos(x + 70, y + HEIGHT);

    con_write_reg->setPos(x + WIDTH - 40, y + HEIGHT);
    con_write->setPos(x + WIDTH - 30, y + HEIGHT);
    con_ctl_write->setPos(x + WIDTH - 20, y + HEIGHT);
}

const Connector *Registers::connector_read1() const {
    return con_read1;
}

const Connector *Registers::connector_read1_reg() const {
    return con_read1_reg;
}

const Connector *Registers::connector_read2() const {
    return con_read2;
}

const Connector *Registers::connector_read2_reg() const {
    return con_read2_reg;
}

const Connector *Registers::connector_write() const {
    return con_write;
}

const Connector *Registers::connector_write_reg() const {
    return con_write_reg;
}

const Connector *Registers::connector_ctl_write() const {
    return con_ctl_write;
}

void Registers::mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event) {
    QGraphicsObject::mouseDoubleClickEvent(event);
    emit open_registers();
}