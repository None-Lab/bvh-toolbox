package main

import (
	"bufio"
	"fmt"
	"io"
	"os"
	"strconv"
	"strings"
	"time"

	"gonum.org/v1/plot"
	"gonum.org/v1/plot/plotter"
	"gonum.org/v1/plot/vg"
)

// WriteJointRotations writes joint rotation data to a CSV file.
func WriteJointRotations(bvhTree *BvhTree, filePath string, endSites bool) bool {
	startTime := time.Now()

	timeCol := make([]float64, bvhTree.NFrames())
	for i := 0; i < bvhTree.NFrames(); i++ {
		timeCol[i] = float64(i) * bvhTree.FrameTime()
	}

	data := make([][]float64, len(bvhTree.GetJoints(endSites)))
	header := []string{"time"}

	for i, joint := range bvhTree.GetJoints(endSites) {
		channels := bvhTree.JointChannels(joint.Name())
		rotationChannels := make([]string, 0)

		for _, channel := range channels {
			if strings.HasSuffix(channel, "rotation") {
				rotationChannels = append(rotationChannels, channel)
			}
		}
		JointChannels := bvhTree.FramesJointChannels(joint.Name(), rotationChannels, 0.0)[i]

		header = append(header, fmt.Sprintf("%s.%s", joint.Name(), strings.ToLower(string(rotationChannels[0][0]))))
		data[i] = append(data[i], timeCol...)
		data[i] = append(data[i], JointChannels...)
	}

	endTime := time.Now()
	fmt.Printf("rotations: %s\n", endTime.Sub(startTime))

	file, err := os.Create(filePath)
	if err != nil {
		fmt.Printf("ERROR(%s): Could not write to file %s.\nMake sure you have writing permissions.\n", err, filePath)
		return false
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	defer writer.Flush()

	fmt.Fprintf(writer, "%s\n", strings.Join(header, ","))
	for _, row := range data {
		for j, v := range row {
			if j > 0 {
				fmt.Fprintf(writer, ",")
			}
			fmt.Fprintf(writer, "%10.5f", v)
		}
		fmt.Fprintf(writer, "\n")
	}

	return true
}

// WriteJointPositions writes joint world position data to a CSV file.
func WriteJointPositions(bvhTree *BvhTree, filePath string, scale float64, endSites bool) bool {
	startTime := time.Now()

	timeCol := make([]float64, bvhTree.NFrames())
	for i := 0; i < bvhTree.NFrames(); i++ {
		timeCol[i] = float64(i) * bvhTree.FrameTime()
	}

	data := make([][]float64, len(bvhTree.GetJoints(true)))
	header := []string{"time"}

	var getWorldPositions func(joint *BvhNode)
	getWorldPositions = func(joint *BvhNode) {
		var offset []float64
		if joint.Value[0] == "End" {
			offset = []float64{0, 0, 0}
		} else {
			offset = bvhTree.JointOffset(joint.Name())
		}
		for _, axis := range []string{"x", "y", "z"} {
			header = append(header, fmt.Sprintf("%s.%s", joint.Name(), axis))
		}

		fmt.Println(bvhTree.NFrames())

		for i := 0; i < bvhTree.NFrames(); i++ {
			fmt.Println(joint.Name())
			fmt.Println(bvhTree.GetJointIndex(joint.Name()))

			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], timeCol[i])
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], scale*offset[0])
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], scale*offset[1])
			data[bvhTree.GetJointIndex(joint.Name())] = append(data[bvhTree.GetJointIndex(joint.Name())], scale*offset[2])
		}
		for _, child := range bvhTree.JointChildren(joint.Name()) {
			if endSites {
				getWorldPositions(child)
			}
			if child.Value[0] == "JOINT" {
				getWorldPositions(child)
			}
		}
	}
	getWorldPositions(bvhTree.Search("ROOT")[0])

	endTime := time.Now()
	fmt.Printf("positions: %s\n", endTime.Sub(startTime))

	file, err := os.Create(filePath)
	if err != nil {
		fmt.Printf("ERROR(%s): Could not write to file %s.\nMake sure you have writing permissions.\n", err, filePath)
		return false
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	defer writer.Flush()

	fmt.Fprintf(writer, "%s\n", strings.Join(header, ","))
	for _, row := range data {
		for j, v := range row {
			if j > 0 {
				fmt.Fprintf(writer, ",")
			}
			fmt.Fprintf(writer, "%10.5f", v)
		}
		fmt.Fprintf(writer, "\n")
	}

	return true
}

// WriteJointHierarchy writes joint hierarchy data to a CSV file.
func WriteJointHierarchy(bvhTree *BvhTree, filePath string, scale float64) bool {
	startTime := time.Now()

	var data [][]string
	for _, joint := range bvhTree.GetJoints(true) {
		jointName := joint.Name()
		parentName := ""
		if bvhTree.JointParent(jointName) != nil {
			parentName = bvhTree.JointParent(jointName).Name()
		}
		offset := bvhTree.JointOffset(jointName)
		row := []string{jointName, parentName, fmt.Sprintf("%f", scale*offset[0]), fmt.Sprintf("%f", scale*offset[1]), fmt.Sprintf("%f", scale*offset[2])}
		data = append(data, row)
	}

	endTime := time.Now()
	fmt.Printf("hierarchy: %s\n", endTime.Sub(startTime))

	file, err := os.Create(filePath)
	if err != nil {
		fmt.Printf("ERROR(%s): Could not write to file %s.\nMake sure you have writing permissions.\n", err, filePath)
		return false
	}
	defer file.Close()

	writer := bufio.NewWriter(file)
	defer writer.Flush()

	fmt.Fprintf(writer, "joint,parent,offset.x,offset.y,offset.z\n")
	for _, row := range data {
		fmt.Fprintf(writer, "%s,%s,%s,%s,%s\n", row[0], row[1], row[2], row[3], row[4])
	}

	return true
}

// Bvh2Csv converts a BVH file to CSV format.
func Bvh2Csv(bvhPath string, dstDirpath string, scale float64, exportRotation bool, exportPosition bool, exportHierarchy bool, endSites bool) bool {
	startTime := time.Now()

	var file *os.File
	var err error
	if file, err = os.Open(bvhPath); err != nil {
		fmt.Printf("ERROR %s: Could not open file %s\n", err, bvhPath)
		return false
	}
	defer file.Close()

	data, _ := io.ReadAll(file)
	bvhTree := NewBvhTree(string(data))

	endTime := time.Now()
	fmt.Printf("file read: %s\n", endTime.Sub(startTime))

	var posSuccess, rotSuccess, hierarchySuccess bool
	posSuccess = false
	rotSuccess = false
	hierarchySuccess = false

	if exportPosition {
		posSuccess = WriteJointPositions(bvhTree, fmt.Sprintf("%s/%s_pos.csv", dstDirpath, bvhPath), scale, endSites)
	}

	if exportRotation {
		rotSuccess = WriteJointRotations(bvhTree, fmt.Sprintf("%s/%s_rot.csv", dstDirpath, bvhPath), endSites)
	}

	if exportHierarchy {
		hierarchySuccess = WriteJointHierarchy(bvhTree, fmt.Sprintf("%s/%s_hierarchy.csv", dstDirpath, bvhPath), scale)
	}

	return bool(posSuccess && rotSuccess && hierarchySuccess)
}

// PlotCsv plots data from a CSV file.
func PlotCsv(filePath string, xColumn int, yColumn int) error {
	file, err := os.Open(filePath)
	if err != nil {
		return err
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)
	scanner.Scan() // Skip header line
	header := strings.Split(scanner.Text(), ",")

	var pts plotter.XYs
	for scanner.Scan() {
		row := strings.Split(scanner.Text(), ",")
		x, err := strconv.ParseFloat(row[xColumn], 64)
		if err != nil {
			return err
		}
		y, err := strconv.ParseFloat(row[yColumn], 64)
		if err != nil {
			return err
		}
		pts = append(pts, plotter.XY{X: x, Y: y})
	}

	p := plot.New()
	s, err := plotter.NewScatter(pts)
	if err != nil {
		return err
	}
	p.Add(s)

	p.X.Label.Text = header[xColumn]
	p.Y.Label.Text = header[yColumn]

	if err := p.Save(4*vg.Inch, 4*vg.Inch, "plot.png"); err != nil {
		return err
	}

	return nil
}

// BvhNode represents a node in a BVH hierarchy.
type BvhNode struct {
	Value    []string
	Children []*BvhNode
	Parent   *BvhNode
	EndSite  bool
}

// AddChild adds a child node to the current node.
func (n *BvhNode) AddChild(item *BvhNode) {
	item.Parent = n
	n.Children = append(n.Children, item)
}

// Filter returns a list of child nodes whose value matches the given key.
func (n *BvhNode) Filter(key string) []*BvhNode {
	var filtered []*BvhNode
	for _, child := range n.Children {
		if child.Value[0] == key {
			filtered = append(filtered, child)
		}
	}
	return filtered
}

// Get returns the value at the given key index.
func (n *BvhNode) Get(key string) []string {
	for _, child := range n.Children {
		for index, item := range child.Value {
			if item == key {
				if index+1 >= len(child.Value) {
					return nil
				}
				return child.Value[index+1:]
			}
		}
	}
	return nil
}

// Name returns the name of the node.
func (n *BvhNode) Name() string {
	return n.Value[1]
}

// String returns a string representation of the node.
func (n *BvhNode) String() string {
	return strings.Join(n.Value, " ")
}

// Bvh represents a BVH file.
type Bvh struct {
	Data   string
	Root   *BvhNode
	Frames [][]string
}

// Tokenize splits the BVH data into tokens.
func (b *Bvh) Tokenize() {
	firstRound := []string{}
	accumulator := ""
	for _, char := range b.Data {
		if char != '\n' && char != '\r' {
			accumulator += string(char)
		} else if accumulator != "" {
			firstRound = append(firstRound, strings.TrimSpace(accumulator))
			accumulator = ""
		}
	}
	if accumulator != "" {
		firstRound = append(firstRound, strings.TrimSpace(accumulator))
		accumulator = ""
	}
	nodeStack := []*BvhNode{b.Root}
	frameTimeFound := false
	node := (*BvhNode)(nil)
	for _, item := range firstRound {
		parts := strings.Split(item, " ")
		if frameTimeFound {
			b.Frames = append(b.Frames, parts)
			continue
		}
		key := parts[0]
		if key == "{" {
			nodeStack = append(nodeStack, node)
		} else if key == "}" {
			nodeStack = nodeStack[:len(nodeStack)-1]
		} else {
			node = &BvhNode{Value: parts}
			nodeStack[len(nodeStack)-1].AddChild(node)
		}
		if key == "Frame" && parts[1] == "Time:" {
			frameTimeFound = true
		}
	}
}

// Search finds nodes matching the given items.
func (b *Bvh) Search(items ...string) []*BvhNode {
	var foundNodes []*BvhNode
	var checkChildren func(node *BvhNode)
	checkChildren = func(node *BvhNode) {
		if len(node.Value) >= len(items) {
			failed := false
			for index, item := range items {
				if node.Value[index] != item {
					failed = true
					break
				}
			}
			if !failed {
				foundNodes = append(foundNodes, node)
			}
		}
		for _, child := range node.Children {
			checkChildren(child)
		}
	}
	checkChildren(b.Root)
	return foundNodes
}

// GetJoints returns a list of joints in the hierarchy.
func (b *Bvh) GetJoints() []*BvhNode {
	var joints []*BvhNode
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint)
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(b.Search("ROOT")[0])
	return joints
}

// GetJointsNames returns a list of joint names.
func (b *Bvh) GetJointsNames() []string {
	var joints []string
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint.Value[1])
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(b.Search("ROOT")[0])
	return joints
}

// JointDirectChildren returns the direct children of a joint.
func (b *Bvh) JointDirectChildren(name string) []*BvhNode {
	joint := b.GetJoint(name)
	return joint.Filter("JOINT")
}

// GetJointIndex returns the index of a joint in the list of joints.
func (b *Bvh) GetJointIndex(name string) int {
	for i, jointName := range b.GetJointsNames() {
		if jointName == name {
			return i
		}
	}
	return -1 // Not found
}

// GetJoint returns the joint with the given name.
func (b *Bvh) GetJoint(name string) *BvhNode {
	found := b.Search("ROOT", name)
	if len(found) == 0 {
		found = b.Search("JOINT", name)
	}
	if len(found) > 0 {
		return found[0]
	}
	return nil
}

// JointOffset returns the offset of a joint.
func (b *Bvh) JointOffset(name string) []float64 {
	joint := b.GetJoint(name)
	offset := joint.Get("OFFSET")
	var result []float64
	for _, item := range offset {
		f, _ := strconv.ParseFloat(item, 64)
		result = append(result, f)
	}
	return result
}

// JointChannels returns the channels of a joint.
func (b *Bvh) JointChannels(name string) []string {
	joint := b.GetJoint(name)
	return joint.Get("CHANNELS")[1:]
}

// GetJointChannelsIndex returns the index of a joint in the list of channels.
func (b *Bvh) GetJointChannelsIndex(jointName string) int {
	index := 0
	for _, joint := range b.GetJoints() {
		if joint.Value[1] == jointName {
			return index
		}
		index += int(joint.Get("CHANNELS")[0][0])
	}
	return -1
}

// GetJointChannelIndex returns the index of a channel in the list of channels.
func (b *Bvh) GetJointChannelIndex(joint string, channel string) int {
	channels := b.JointChannels(joint)
	for i, c := range channels {
		if c == channel {
			return i
		}
	}
	return -1 // Not found
}

// FrameJointChannel returns the value of a channel for a joint at a given frame.
func (b *Bvh) FrameJointChannel(frameIndex int, joint string, channel string, value float64) float64 {
	jointIndex := b.GetJointChannelsIndex(joint)
	channelIndex := b.GetJointChannelIndex(joint, channel)
	if channelIndex == -1 && value != 0.0 {
		return value
	}
	if frameIndex >= len(b.Frames) {
		return 0.0
	}
	if jointIndex+channelIndex >= len(b.Frames[frameIndex]) {
		return 0.0
	}
	f, _ := strconv.ParseFloat(b.Frames[frameIndex][jointIndex+channelIndex], 64)
	return f
}

// FrameJointChannels returns the values of multiple channels for a joint at a given frame.
func (b *Bvh) FrameJointChannels(frameIndex int, joint string, channels []string, value float64) []float64 {
	var values []float64
	jointIndex := b.GetJointChannelsIndex(joint)
	for _, channel := range channels {
		channelIndex := b.GetJointChannelIndex(joint, channel)
		if channelIndex == -1 && value != 0.0 {
			values = append(values, value)
		} else {
			if frameIndex >= len(b.Frames) {
				values = append(values, 0.0)
			} else if jointIndex+channelIndex >= len(b.Frames[frameIndex]) {
				values = append(values, 0.0)
			} else {
				f, _ := strconv.ParseFloat(b.Frames[frameIndex][jointIndex+channelIndex], 64)
				values = append(values, f)
			}
		}
	}
	return values
}

// FramesJointChannels returns the values of multiple channels for a joint across all frames.
func (b *Bvh) FramesJointChannels(joint string, channels []string, value float64) [][]float64 {
	var allFrames [][]float64
	jointIndex := b.GetJointChannelsIndex(joint)
	for _, frame := range b.Frames {
		var values []float64
		for _, channel := range channels {
			channelIndex := b.GetJointChannelIndex(joint, channel)
			if channelIndex == -1 && value != 0.0 {
				values = append(values, value)
			} else {
				if jointIndex+channelIndex >= len(frame) {
					values = append(values, 0.0)
				} else {
					f, _ := strconv.ParseFloat(frame[jointIndex+channelIndex], 64)
					values = append(values, f)
				}
			}
		}
		allFrames = append(allFrames, values)
	}
	return allFrames
}

// JointParent returns the parent of a joint.
func (b *Bvh) JointParent(name string) *BvhNode {
	joint := b.GetJoint(name)
	if joint.Parent == b.Root {
		return nil
	}
	return joint.Parent
}

// JointParentIndex returns the index of the parent of a joint.
func (b *Bvh) JointParentIndex(name string) int {
	joint := b.GetJoint(name)
	if joint.Parent == b.Root {
		return -1
	}
	for i, parentCandidate := range b.GetJoints() {
		if parentCandidate == joint.Parent {
			return i
		}
	}
	return -1 // Should never happen if the parent exists
}

// NFrames returns the number of frames in the BVH file.
func (b *Bvh) NFrames() int {
	framesNode := b.Search("Frames:")[0]
	nFrames, _ := strconv.Atoi(framesNode.Value[1])
	return nFrames
}

// FrameTime returns the frame time of the BVH file.
func (b *Bvh) FrameTime() float64 {
	frameTimeNode := b.Search("Frame", "Time:")[0]
	frameTime, _ := strconv.ParseFloat(frameTimeNode.Value[2], 64)
	return frameTime
}

// BvhTree extends the Bvh class with additional functionality.
type BvhTree struct {
	*Bvh
}

// NewBvhTree creates a new BvhTree from the given BVH data.
func NewBvhTree(data string) *BvhTree {
	bvh := &Bvh{Data: data, Root: &BvhNode{}}
	bvh.Tokenize()
	return &BvhTree{Bvh: bvh}
}

// GetJointsNames returns a list of joint names, including End Sites.
func (bt *BvhTree) GetJointsNames(endSites bool) []string {
	var joints []string
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint.Value[1])
		if endSites {
			for _, end := range joint.Filter("End") {
				joints = append(joints, end.Value[1])
			}
		}
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(bt.Search("ROOT")[0])
	return joints
}

// GetJoints returns a list of joints, including End Sites.
func (bt *BvhTree) GetJoints(endSites bool) []*BvhNode {
	var joints []*BvhNode
	var iterateJoints func(joint *BvhNode)
	iterateJoints = func(joint *BvhNode) {
		joints = append(joints, joint)
		if endSites {
			for _, end := range joint.Filter("End") {
				joints = append(joints, end)
			}
		}
		for _, child := range joint.Filter("JOINT") {
			iterateJoints(child)
		}
	}
	iterateJoints(bt.Search("ROOT")[0])
	return joints
}

// GetJoint returns the joint with the given name, including End Sites.
func (bt *BvhTree) GetJoint(name string) *BvhNode {
	found := bt.Search("ROOT", name)
	if len(found) == 0 {
		found = bt.Search("JOINT", name)
	}
	if len(found) == 0 {
		found = bt.Search("End", name)
	}
	if len(found) > 0 {
		return found[0]
	}
	return nil
}

// GetJointIndex returns the index of a joint in the list of joints, including End Sites.
func (bt *BvhTree) GetJointIndex(name string) int {
	joint := bt.GetJoint(name)
	if joint.Parent == bt.Root {
		return -1
	}
	for i, parentCandidate := range bt.GetJoints(true) { // Get all joints, including End Sites
		if parentCandidate == joint.Parent {
			return i
		}
	}
	return -1 // Should never happen if the parent exists
}

// JointChildren returns the direct children of a joint, including End Sites.
func (bt *BvhTree) JointChildren(name string) []*BvhNode {
	joint := bt.GetJoint(name)
	var children []*BvhNode
	for _, child := range joint.Filter("JOINT") {
		children = append(children, child)
	}
	for _, child := range joint.Filter("End") {
		children = append(children, child)
	}
	return children
}

// Write writes the BVH data to the given stream.
func (bt *BvhTree) Write(out io.Writer) {
	fmt.Fprintf(out, "%s\n", bt.GetHierarchyString())
	fmt.Fprintf(out, "%s\n", bt.GetMotionString())
}

// GetHierarchyString returns the hierarchy string of the BVH data.
func (bt *BvhTree) GetHierarchyString() string {
	var s string
	s += "HIERARCHY\n"
	for _, joint := range bt.GetJoints(true) {
		s = bt.CloseScopes(s, bt.GetJointDepth(joint.Name()))
		s += bt.GetJointString(joint)
	}
	s = bt.CloseScopes(s, 0)
	return s
}

// GetJointDepth returns the depth of a joint in the hierarchy.
func (bt *BvhTree) GetJointDepth(name string) int {
	depth := 0
	parent := bt.JointParent(name)
	for parent != nil {
		depth++
		parent = bt.JointParent(parent.Name())
	}
	return depth
}

// GetJointString returns the string representation of a joint.
func (bt *BvhTree) GetJointString(joint *BvhNode) string {
	depth := bt.GetJointDepth(joint.Name())
	var s string
	if len(bt.JointChildren(joint.Name())) == 0 {
		s += fmt.Sprintf("%sEnd Site\n", strings.Repeat("  ", depth))
		s += fmt.Sprintf("%s{{\n", strings.Repeat("  ", depth))
		s += fmt.Sprintf("%sOFFSET %s\n", strings.Repeat("  ", depth+1), strings.Join(joint.Get("OFFSET"), " "))
		s += fmt.Sprintf("%s}}\n", strings.Repeat("  ", depth))
	} else {
		s += fmt.Sprintf("%s%s\n", strings.Repeat("  ", depth), joint.String())
		s += fmt.Sprintf("%s{{\n", strings.Repeat("  ", depth))
		for _, attribute := range []string{"OFFSET", "CHANNELS"} {
			s += fmt.Sprintf("%s%s %s\n", strings.Repeat("  ", depth+1), attribute, strings.Join(joint.Get(attribute), " "))
		}
	}
	return s
}

// CloseScopes closes open scopes in the hierarchy string.
func (bt *BvhTree) CloseScopes(hierarchyString string, targetDepth int) string {
	lastDepth := strings.Count(strings.Split(hierarchyString, "\n")[len(strings.Split(hierarchyString, "\n"))-2], "  ")
	diff := lastDepth - targetDepth
	for depth := 0; depth < diff; depth++ {
		hierarchyString += fmt.Sprintf("%s}}\n", strings.Repeat("  ", lastDepth-depth-1))
	}
	return hierarchyString
}

// GetMotionString returns the motion string of the BVH data.
func (bt *BvhTree) GetMotionString() string {
	var s string
	s += "MOTION\n"
	s += fmt.Sprintf("Frames: %d\n", bt.NFrames())
	s += fmt.Sprintf("Frame Time: %f\n", bt.FrameTime())
	for _, frame := range bt.Frames {
		s += strings.Join(frame, " ")
		s += "\n"
	}
	return s
}

// contains returns true if the given slice contains the given element.
func contains(s []string, e string) bool {
	for _, a := range s {
		if a == e {
			return true
		}
	}
	return false
}

// index returns the index of the given element in the given slice.
func index(s []string, e string) int {
	for i, a := range s {
		if a == e {
			return i
		}
	}
	return -1
}

func main() {
	// // Example usage:
	// bvhData := ``

	// // Create a new BvhTree from the BVH data.
	// bvhTree := NewBvhTree(bvhData)

	// // Print the joint names, including End Sites.
	// fmt.Println("Joint Names:", bvhTree.GetJointsNames(true))

	// // Print the number of frames.
	// fmt.Println("Number of Frames:", bvhTree.NFrames())

	// // Print the frame time.
	// fmt.Println("Frame Time:", bvhTree.FrameTime())

	// // Get the values of the Xrotation channel for the LeftHip joint for all frames.
	// leftHipXRotation := bvhTree.FramesJointChannels("LeftHip", []string{"Xrotation"}, 0.0)
	// fmt.Println("LeftHip Xrotation:", leftHipXRotation)

	// Example usage:
	bvhPath := "test/ex/test.bvh"
	dstDirpath := "test/out"

	// Convert BVH to CSV
	if success := Bvh2Csv(bvhPath, dstDirpath, 1.0, true, true, true, true); success {
		fmt.Println("BVH data converted to CSV successfully.")
	}

	// // Plot CSV data (example: plot the x-position of the Hips joint)
	// if err := PlotCsv(fmt.Sprintf("%s/%s_pos.csv", dstDirpath, strings.TrimSuffix(bvhPath, ".bvh")), 0, 1); err != nil {
	// 	fmt.Println("Error plotting data:", err)
	// }

}
